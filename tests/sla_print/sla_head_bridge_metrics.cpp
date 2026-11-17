#include "sla_head_bridge_metrics.hpp"

#include <algorithm>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <utility>

namespace Slic3r { namespace sla {

namespace {

using clearance::Prim;
using clearance::PrimKind;

bool same(const Vec3d &a, const Vec3d &b) { return (a - b).norm() < 1e-9; }

bool is_path_bar(PrimKind k) { return k == PrimKind::head_bar || k == PrimKind::route_bar; }

// A builder bar as the metrics see it.
struct Bar
{
    Vec3d    s, e;
    double   r;
    PrimKind kind;
    long     a, b;
    long     path = -1; // head whose path it is
};

// Union-find over pillar ids; the lower id is the root, like the planner's.
struct Clusters
{
    std::vector<size_t> parent;
    size_t find(size_t i) const { while (parent[i] != i) i = parent[i]; return i; }
    void unite(size_t i, size_t j)
    {
        size_t ri = find(i), rj = find(j);
        if (ri == rj) return;
        if (rj < ri) std::swap(ri, rj);
        parent[rj] = ri;
    }
};

// Which count a pair of elements falls in. `x_aux`, `y_aux` tell an auxiliary
// pillar apart from the rest.
enum class Cat { none, A, A_prop, C_pillar, C_head, revC_aux, revC_head };

Cat category(const Prim &x, bool x_aux, const Prim &y, bool y_aux)
{
    const bool aux_x = (x.kind == PrimKind::pillar && x_aux) || x.kind == PrimKind::prop_bar;
    const bool aux_y = (y.kind == PrimKind::pillar && y_aux) || y.kind == PrimKind::prop_bar;

    if (is_path_bar(x.kind) && is_path_bar(y.kind)) return Cat::A;
    if ((is_path_bar(x.kind) && y.kind == PrimKind::prop_bar) ||
        (is_path_bar(y.kind) && x.kind == PrimKind::prop_bar))
        return Cat::A_prop;
    if (aux_x || aux_y) return Cat::revC_aux;
    if (is_path_bar(x.kind) || is_path_bar(y.kind)) {
        const PrimKind other = is_path_bar(x.kind) ? y.kind : x.kind;
        return other == PrimKind::pillar ? Cat::C_pillar : Cat::C_head;
    }
    // What is left judged: an ordinary pillar and a head.
    return Cat::revC_head;
}

// The capsule's box, grown by `grow`, for the sweep below.
struct Box { Vec3d lo, hi; };

Box box_of(const clearance::Capsule &c, double grow)
{
    return {(c.a.cwiseMin(c.b).array() - (c.r + grow)).matrix(),
            (c.a.cwiseMax(c.b).array() + (c.r + grow)).matrix()};
}

bool overlap(const Box &p, const Box &q)
{
    return (p.lo.array() <= q.hi.array()).all() && (q.lo.array() <= p.hi.array()).all();
}

// Every pair of boxes that overlap, by a sweep along x.
template<class Fn> void each_near_pair(const std::vector<Box> &boxes, Fn &&fn)
{
    std::vector<size_t> order(boxes.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(),
              [&](size_t i, size_t j) { return boxes[i].lo.x() < boxes[j].lo.x(); });
    for (size_t k = 0; k < order.size(); ++k) {
        const Box &bi = boxes[order[k]];
        for (size_t m = k + 1; m < order.size(); ++m) {
            const Box &bj = boxes[order[m]];
            if (bj.lo.x() > bi.hi.x()) break;
            if (overlap(bi, bj)) fn(std::min(order[k], order[m]), std::max(order[k], order[m]));
        }
    }
}

} // namespace

HeadBridgeMetrics head_bridge_metrics(const SupportTreeBuilder &bld, const ClearanceReport &report)
{
    using namespace clearance;

    HeadBridgeMetrics m;
    const double c = HeadBridgeMetrics::SAFETY_CLEARANCE_MM;

    m.l1                = report.l1;
    m.l2                = report.l2;
    m.none              = report.none;
    m.centroid_recorded = report.centroid.size();
    m.centroid_native   = report.native;

    std::map<long, const Head *> heads; // the valid ones, by id
    for (const Head &h : bld.heads())
        if (h.is_valid())
            heads[h.id] = &h;
    m.heads_valid = heads.size();

    const auto &pillars = bld.pillars();
    m.pillars = pillars.size();
    for (const Pillar &p : pillars)
        m.aux_pillars += p.props_for != SupportTreeNode::ID_UNSET;

    // ── bars, and the head whose path each is ──
    std::vector<Bar> bars;
    for (const Bridge &br : bld.bridges()) {
        PrimKind k;
        switch (br.kind) {
        case BridgeKind::head:  k = PrimKind::head_bar;  ++m.head_bars;  break;
        case BridgeKind::route: k = PrimKind::route_bar; ++m.route_bars; break;
        case BridgeKind::prop:  k = PrimKind::prop_bar;  ++m.prop_bars;  break;
        default: continue; // a truss bar among the bridges is not made
        }
        bars.push_back({br.startp, br.endp, br.r, k, br.owner_a, br.owner_b});
    }
    for (const DiffBridge &br : bld.diffbridges()) {
        ++m.route_bars;
        bars.push_back({br.startp, br.endp, std::max(br.r, br.end_r), PrimKind::route_bar,
                        br.owner_a, br.owner_b});
    }
    m.truss_bars = bld.crossbridges().size();

    for (Bar &bar : bars) {
        if (!is_path_bar(bar.kind)) continue;
        if (bar.kind == PrimKind::head_bar && bar.a >= 0 && pillars[size_t(bar.a)].starts_from_head)
            bar.path = pillars[size_t(bar.a)].start_junction_id;
        else
            for (const auto &[id, h] : heads)
                if (same(h->junction_point(), bar.s)) { bar.path = id; break; }
    }
    // A route chains on from where a route bar reached, and from nothing else:
    // a head bar ends on a pillar, where another head's route may start.
    for (bool more = true; more;) {
        more = false;
        for (Bar &bar : bars)
            if (bar.kind == PrimKind::route_bar && bar.path < 0)
                for (const Bar &prev : bars)
                    if (prev.kind == PrimKind::route_bar && prev.path >= 0 && same(prev.e, bar.s)) {
                        bar.path = prev.path;
                        more     = true;
                        break;
                    }
    }
    for (const Bar &bar : bars)
        m.unassigned += is_path_bar(bar.kind) && bar.path < 0;

    auto pillar_path = [&](const Pillar &p) -> long {
        if (p.props_for != SupportTreeNode::ID_UNSET || p.frozen) return -1;
        if (p.starts_from_head) return p.start_junction_id;
        for (const auto &[id, h] : heads)
            if (h->pillar_id == p.id) return id;
        for (const Bar &bar : bars) // a head bar reaching it is another head's
            if (bar.kind == PrimKind::route_bar && bar.b == p.id) return bar.path;
        return -1;
    };

    // ── the elements as the clearance rules see them ──
    std::vector<Prim> prims;
    std::vector<bool> aux; // per prim: an auxiliary pillar
    std::map<long, Prim> pinheads;
    for (const auto &[id, h] : heads) {
        // The pillar carrying it: the one it sits on, else the one its route
        // leaves the junction for, else the one its route comes down on.
        long carrier = h->pillar_id;
        for (const Bar &bar : bars)
            if (carrier < 0 && bar.path == id && bar.b >= 0 && same(bar.s, h->junction_point()))
                carrier = bar.b;
        for (const Bar &bar : bars)
            if (carrier < 0 && bar.path == id && bar.b >= 0)
                carrier = bar.b;
        Prim p;
        p.cap      = pinhead_capsule(h->pos, h->junction_point(), h->r_back_mm);
        p.kind     = PrimKind::pinhead;
        p.head_id  = id;
        p.pillar_a = carrier;
        prims.push_back(p);
        aux.push_back(false);
        pinheads[id] = p;
    }
    for (const Pillar &pl : pillars) {
        Prim p;
        p.cap      = pillar_capsule(pl.endpt, pl.height, pl.r_start, pl.r_end);
        p.kind     = PrimKind::pillar;
        p.head_id  = pillar_path(pl);
        p.pillar_a = pl.id;
        prims.push_back(p);
        aux.push_back(pl.props_for != SupportTreeNode::ID_UNSET);
    }
    const size_t first_bar = prims.size();
    for (const Bar &bar : bars) {
        Prim p;
        p.cap      = bar_capsule(bar.s, bar.e, bar.r);
        p.kind     = bar.kind;
        p.head_id  = bar.path;
        p.pillar_a = bar.a;
        p.pillar_b = bar.b;
        prims.push_back(p);
        aux.push_back(false);
    }

    // What the records name: a degraded route and what it kept too close, a
    // centre pillar's head and what it came too close to. Two heads by their
    // ids; a pillar no head's path owns (with_head -1) by its own id, never by
    // -1, which would name every element no path owns.
    std::set<std::pair<long, long>> named;        // (head, head), ascending
    std::set<std::pair<long, long>> named_pillar; // (head, unowned pillar)
    auto name = [&](long head, const Contact &ct) {
        if (head < 0) return;
        if (ct.with_head >= 0)
            named.insert({std::min(head, ct.with_head), std::max(head, ct.with_head)});
        else if (ct.with == PrimKind::pillar && ct.with_pillar >= 0)
            named_pillar.insert({head, ct.with_pillar});
    };
    for (const RouteRecord &rec : report.routes)
        for (const Contact &ct : rec.contacts)
            name(rec.head, ct);
    for (const CentroidRecord &rec : report.centroid)
        if (rec.pillar >= 0 && size_t(rec.pillar) < pillars.size())
            name(pillar_path(pillars[size_t(rec.pillar)]), rec.contact);

    auto is_named = [&](const Prim &x, const Prim &y) {
        if (x.head_id >= 0 && y.head_id >= 0)
            return named.count({std::min(x.head_id, y.head_id), std::max(x.head_id, y.head_id)}) > 0;
        const Prim &own = x.head_id >= 0 ? x : y, &other = x.head_id >= 0 ? y : x;
        return own.head_id >= 0 && other.kind == PrimKind::pillar &&
               named_pillar.count({own.head_id, other.pillar_a}) > 0;
    };

    const PillarAxisFn axis_of = [&bld](long id) {
        const Pillar &p = bld.pillar(id);
        return JointAxis{Vec2d{p.endpt.x(), p.endpt.y()}, std::max(p.r_start, p.r_end)};
    };

    auto note_unrecorded = [&m](const char *cat, const Prim &x, const Prim &y, double dist,
                           double required, const Vec3d &at) {
        ++m.unrecorded;
        if (m.unrecorded_list.size() >= 20) return;
        std::ostringstream os;
        os << std::fixed << std::setprecision(3) << cat << ": " << kind_name(x.kind) << " of head "
           << x.head_id << " x " << kind_name(y.kind) << " of head " << y.head_id << ": dist "
           << dist << " < " << required << " at (" << at.x() << "," << at.y() << "," << at.z()
           << ")";
        m.unrecorded_list.push_back(os.str());
    };

    // ── A, C, reverse C ──
    std::vector<Box> boxes;
    boxes.reserve(prims.size());
    for (const Prim &p : prims) boxes.push_back(box_of(p.cap, c / 2.));

    each_near_pair(boxes, [&](size_t i, size_t j) {
        const Prim &x = prims[i], &y = prims[j];
        Contact     ct;
        if (!pair_contact(x, y, c, axis_of, ct)) return;

        const Cat cat = category(x, aux[i], y, aux[j]);
        switch (cat) {
        case Cat::A:        ++m.A; break;
        case Cat::A_prop:   ++m.A_prop; break;
        case Cat::C_pillar: ++m.C_pillar; break;
        case Cat::C_head:   ++m.C_head; break;
        case Cat::revC_aux: ++m.revC_aux; break;
        case Cat::revC_head: {
            ++m.revC_head;
            // A pillar against a head natively that close to the head that
            // grew the pillar: support point spacing, out of scope. Only the
            // head it grew from (starts_from_head), as the spec says - not a
            // head sitting on it or a route coming down on it.
            const Prim   &pl  = x.kind == PrimKind::pillar ? x : y;
            const Prim   &ph  = x.kind == PrimKind::pinhead ? x : y;
            const Pillar &own = pillars[size_t(pl.pillar_a)];
            const auto    grower =
                own.starts_from_head ? pinheads.find(own.start_junction_id) : pinheads.end();
            if (grower != pinheads.end() &&
                capsule_dist(grower->second.cap, ph.cap) < grower->second.cap.r + ph.cap.r + c) {
                ++m.revC_head_native;
                return;
            }
            ++m.revC_head_net;
            break;
        }
        case Cat::none: return;
        }

        static const char *names[] = {"", "A", "A_prop", "C_pillar", "C_head", "revC_aux",
                                      "revC_head"};
        if (cat == Cat::A_prop || cat == Cat::revC_aux) {
            note_unrecorded(names[int(cat)], x, y, ct.dist, ct.required, ct.p_new);
            return;
        }
        if (is_named(x, y))
            ++m.recorded;
        else
            note_unrecorded(names[int(cat)], x, y, ct.dist, ct.required, ct.p_new);
    });

    // ── B: path bars x truss bars, judged the way the bracing planner does ──
    Clusters cl;
    cl.parent.resize(pillars.size());
    for (size_t i = 0; i < pillars.size(); ++i) cl.parent[i] = i;
    for (size_t i = 0; i < pillars.size(); ++i)
        for (size_t j = i + 1; j < pillars.size(); ++j) {
            const Pillar &p = pillars[i], &q = pillars[j];
            const Vec2d   d{p.endpt.x() - q.endpt.x(), p.endpt.y() - q.endpt.y()};
            if (d.norm() < p.r_start + q.r_start + c) cl.unite(i, j);
        }
    auto valid = [&pillars](long id) { return id >= 0 && size_t(id) < pillars.size(); };

    for (const Bridge &tb : bld.crossbridges()) {
        const Capsule ty = bar_capsule(tb.startp, tb.endp, tb.r);
        const Box     by = box_of(ty, c / 2.);
        for (size_t i = first_bar; i < prims.size(); ++i) {
            const Prim &x = prims[i];
            if (!is_path_bar(x.kind) || !overlap(boxes[i], by)) continue;

            // The pillars of the truss's ends' clusters this bar ends on.
            std::vector<JointAxis> joints;
            for (long id : {x.pillar_a, x.pillar_b}) {
                if (!valid(id) || (id == x.pillar_b && x.pillar_b == x.pillar_a)) continue;
                const size_t root = cl.find(size_t(id));
                if ((valid(tb.owner_a) && cl.find(size_t(tb.owner_a)) == root) ||
                    (valid(tb.owner_b) && cl.find(size_t(tb.owner_b)) == root))
                    joints.push_back(axis_of(id));
            }
            const double d = joints.empty() ? capsule_dist(x.cap, ty) :
                                              joint_clipped_dist(x.cap, ty, joints);
            if (!(d < x.cap.r + ty.r + c)) continue;
            ++m.B;
            m.B_shared += !joints.empty();
            if (m.unrecorded_list.size() < 20) {
                std::ostringstream os;
                os << std::fixed << std::setprecision(3) << "B: " << kind_name(x.kind)
                   << " of head " << x.head_id << " x truss " << tb.owner_a << "-" << tb.owner_b
                   << ": dist " << d << " < " << x.cap.r + ty.r + c
                   << (joints.empty() ? "" : " (outside R)");
                m.unrecorded_list.push_back(os.str());
            }
            ++m.unrecorded;
        }
    }

    // ── D: more than one way down for a head ──
    // Every head bar of its path, wherever it starts (a bar to a pillar starts
    // at the bottom of the head's own short pillar); every route bar leaving
    // its junction; and a pillar it grew that no head bar starts from (its own
    // pillar to the ground, or to an anchor). A short pillar under head bars is
    // the start of those bars, not a way of its own.
    std::set<long> stems;
    for (const Bar &bar : bars)
        if (bar.kind == PrimKind::head_bar && bar.a >= 0) stems.insert(bar.a);
    for (const auto &[id, h] : heads) {
        const Vec3d jp   = h->junction_point();
        size_t      ways = 0;
        for (const Bar &bar : bars)
            ways += bar.path == id && (bar.kind == PrimKind::head_bar ||
                                       (bar.kind == PrimKind::route_bar && same(bar.s, jp)));
        for (const Pillar &p : pillars)
            ways += p.starts_from_head && p.start_junction_id == id && !stems.count(p.id);
        m.D += ways > 1;
    }

    // ── a thick head onto a thin pillar ──
    for (const Bar &bar : bars)
        if (bar.kind == PrimKind::head_bar && valid(bar.b) && heads.count(bar.path) &&
            pillars[size_t(bar.b)].r_start < heads.at(bar.path)->r_back_mm - 1e-9)
            ++m.thick_on_thin;

    // ── truss bars laid twice, and pairs beyond the pillars reached ──
    std::map<std::pair<long, long>, std::vector<std::pair<Vec3d, Vec3d>>> by_pair;
    for (const Bridge &tb : bld.crossbridges()) {
        if (!valid(tb.owner_a) || !valid(tb.owner_b) || tb.owner_a == tb.owner_b) continue;
        by_pair[{std::min(tb.owner_a, tb.owner_b), std::max(tb.owner_a, tb.owner_b)}]
            .push_back({tb.startp, tb.endp});
    }
    std::map<long, std::set<long>> partners;
    for (const auto &[pair, segs] : by_pair) {
        partners[pair.first].insert(pair.second);
        partners[pair.second].insert(pair.first);
        bool dup = false;
        for (size_t i = 0; i < segs.size() && !dup; ++i)
            for (size_t j = i + 1; j < segs.size() && !dup; ++j)
                dup = (same(segs[i].first, segs[j].first) && same(segs[i].second, segs[j].second)) ||
                      (same(segs[i].first, segs[j].second) && same(segs[i].second, segs[j].first));
        m.dup_truss_pairs += dup;
    }
    for (const Pillar &p : pillars) {
        // A frozen pillar's pairs count bracing grown by earlier generations,
        // which is not in this builder.
        if (p.frozen) continue;
        const auto   it      = partners.find(p.id);
        const size_t reached = it == partners.end() ? 0 : it->second.size();
        if (p.pairs > reached) m.pairs_excess += p.pairs - reached;
    }

    return m;
}

std::string HeadBridgeMetrics::to_json() const
{
    std::ostringstream o;
    o << "{\"heads_valid\":" << heads_valid << ",\"pillars\":" << pillars
      << ",\"aux_pillars\":" << aux_pillars << ",\"head_bars\":" << head_bars
      << ",\"route_bars\":" << route_bars << ",\"prop_bars\":" << prop_bars
      << ",\"truss_bars\":" << truss_bars << ",\"unassigned\":" << unassigned
      << ",\"A\":" << A << ",\"A_prop\":" << A_prop << ",\"B\":" << B
      << ",\"B_shared\":" << B_shared << ",\"C_pillar\":" << C_pillar
      << ",\"C_head\":" << C_head << ",\"revC_aux\":" << revC_aux
      << ",\"revC_head\":" << revC_head << ",\"revC_head_native\":" << revC_head_native
      << ",\"revC_head_net\":" << revC_head_net << ",\"recorded\":" << recorded
      << ",\"unrecorded\":" << unrecorded << ",\"D\":" << D
      << ",\"thick_on_thin\":" << thick_on_thin << ",\"dup_truss_pairs\":" << dup_truss_pairs
      << ",\"pairs_excess\":" << pairs_excess << ",\"l1\":" << l1 << ",\"l2\":" << l2
      << ",\"none\":" << none << ",\"centroid_recorded\":" << centroid_recorded
      << ",\"centroid_native\":" << centroid_native << ",\"unrecorded_list\":[";
    for (size_t i = 0; i < unrecorded_list.size(); ++i) {
        o << (i ? "," : "") << '"';
        for (char ch : unrecorded_list[i]) o << (ch == '"' || ch == '\\' ? "\\" : "") << ch;
        o << '"';
    }
    o << "]}";
    return o.str();
}

}} // namespace Slic3r::sla

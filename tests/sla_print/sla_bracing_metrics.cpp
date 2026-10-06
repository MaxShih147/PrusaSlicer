#include "sla_bracing_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <sstream>

namespace Slic3r { namespace sla {

namespace {

using V2 = Eigen::Vector2d;

double cross2(const V2 &a, const V2 &b) { return a.x() * b.y() - a.y() * b.x(); }

double point_seg_dist2d(const V2 &p, const V2 &a, const V2 &b)
{
    const V2     ab = b - a;
    const double l2 = ab.squaredNorm();
    if (l2 < 1e-12) return (p - a).norm();
    const double t = std::clamp((p - a).dot(ab) / l2, 0., 1.);
    return (p - (a + t * ab)).norm();
}

// Proper and touching intersections both give 0; otherwise the closest
// endpoint-to-segment distance, which is the minimum for non-crossing segments.
double seg_seg_dist2d(const V2 &a, const V2 &b, const V2 &c, const V2 &d)
{
    const double d1 = cross2(b - a, c - a), d2 = cross2(b - a, d - a);
    const double d3 = cross2(d - c, a - c), d4 = cross2(d - c, b - c);
    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
        ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
        return 0.;

    return std::min({point_seg_dist2d(a, c, d), point_seg_dist2d(b, c, d),
                     point_seg_dist2d(c, a, b), point_seg_dist2d(d, a, b)});
}

struct Group
{
    long   a, b;  // pillar ids, a < b
    double r = 0; // the thickest bar of the group
    bool   relaxed = false;
    bool   fallback = false;
    V2     pa, pb;
    double minx, miny, maxx, maxy;
};

// Union-find over pillar indices; the lower index is the root.
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
    bool same(long a, long b) const { return a == b || find(size_t(a)) == find(size_t(b)); }
};

bool valid_pillar(const SupportTreeBuilder &bld, long id)
{
    return id >= 0 && size_t(id) < bld.pillars().size();
}

struct Fnv
{
    uint64_t h = 1469598103934665603ull;
    void bytes(const void *p, size_t n)
    {
        auto *c = static_cast<const unsigned char *>(p);
        for (size_t i = 0; i < n; ++i) { h ^= c[i]; h *= 1099511628211ull; }
    }
    template<class T> void pod(const T &v) { bytes(&v, sizeof(T)); }
    void v3(const Vec3d &v) { pod(v.x()); pod(v.y()); pod(v.z()); }
    std::string hex() const
    {
        char buf[17];
        std::snprintf(buf, sizeof buf, "%016llx", (unsigned long long) h);
        return buf;
    }
};

template<class B> void hash_bridge(Fnv &f, const B &b)
{
    f.pod(b.id); f.pod(b.r); f.v3(b.startp); f.v3(b.endp);
    f.pod(b.owner_a); f.pod(b.owner_b);
}

} // namespace

BracingMetrics bracing_metrics(const SupportTreeBuilder &bld, double ground_z)
{
    BracingMetrics m;
    const double   c       = BracingMetrics::SAFETY_CLEARANCE_MM;
    const double   cos_min = std::cos(BracingMetrics::MIN_SHARED_ANGLE_DEG * PI / 180.);
    const auto    &pillars = bld.pillars();

    for (const Head &h : bld.heads()) m.heads_valid += h.is_valid();

    m.pillars = pillars.size();
    for (const Pillar &p : pillars) {
        m.frozen      += p.frozen;
        m.aux_pillars += p.props_for != SupportTreeNode::ID_UNSET;
    }

    // ── bars by origin ──
    std::map<std::pair<long, long>, Group> groups;
    auto check_kind = [&m](const Bridge &br, BridgeKind expected) {
        m.kind_unset     += br.kind == BridgeKind::unset;
        m.kind_disagrees += br.kind != BridgeKind::unset && br.kind != expected;
    };

    for (const Bridge &br : bld.crossbridges()) {
        ++m.truss_bars;
        check_kind(br, BridgeKind::truss);
        if (!valid_pillar(bld, br.owner_a) || !valid_pillar(bld, br.owner_b) ||
            br.owner_a == br.owner_b) {
            ++m.orphan_truss;
            continue;
        }
        const long a = std::min(br.owner_a, br.owner_b);
        const long b = std::max(br.owner_a, br.owner_b);
        Group &g = groups[{a, b}];
        g.a = a; g.b = b;
        g.r = std::max(g.r, br.r);
        g.relaxed = g.relaxed || br.relaxed;
        g.fallback = g.fallback || br.fallback;
    }
    for (const Bridge &br : bld.bridges()) {
        // A bar naming no pillar is a route's. One naming a pillar is a prop's
        // when that pillar is a prop; otherwise it is a head's, or the last bar
        // of a route, which names the pillar it ends on. Nothing but the
        // recorded kind tells those two apart (a head's bar may end on a
        // pillar's top too), so only there the kind is taken, not checked.
        const bool owned_route = valid_pillar(bld, br.owner_b) &&
                                 pillars[size_t(br.owner_b)].props_for == SupportTreeNode::ID_UNSET &&
                                 br.kind == BridgeKind::route;
        if (!valid_pillar(bld, br.owner_b) || owned_route) {
            ++m.route_bars;
            check_kind(br, BridgeKind::route);
        } else if (pillars[size_t(br.owner_b)].props_for != SupportTreeNode::ID_UNSET) {
            ++m.prop_bars;
            check_kind(br, BridgeKind::prop);
        } else {
            ++m.head_bars;
            check_kind(br, BridgeKind::head);
        }
    }
    for (const DiffBridge &br : bld.diffbridges()) check_kind(br, BridgeKind::route);
    m.route_diff   = bld.diffbridges().size();
    m.truss_groups = groups.size();

    std::vector<Group> gs;
    gs.reserve(groups.size());
    double rmax_bar = 0., rmax_pillar = 0.;
    for (auto &[key, g] : groups) {
        g.pa = pillars[size_t(g.a)].endpoint().head<2>();
        g.pb = pillars[size_t(g.b)].endpoint().head<2>();
        g.minx = std::min(g.pa.x(), g.pb.x()); g.maxx = std::max(g.pa.x(), g.pb.x());
        g.miny = std::min(g.pa.y(), g.pb.y()); g.maxy = std::max(g.pa.y(), g.pb.y());
        rmax_bar = std::max(rmax_bar, g.r);
        m.relaxed_groups += g.relaxed;
        m.fallback_groups += g.fallback;
        gs.emplace_back(g);
    }
    for (const Pillar &p : pillars) rmax_pillar = std::max(rmax_pillar, p.r_start);

    // ── clusters of hugging pillars ──
    std::vector<size_t> porder(pillars.size());
    for (size_t i = 0; i < porder.size(); ++i) porder[i] = i;
    std::sort(porder.begin(), porder.end(), [&pillars](size_t l, size_t r) {
        return pillars[l].endpoint().x() < pillars[r].endpoint().x();
    });

    Clusters cl;
    cl.parent.resize(pillars.size());
    for (size_t i = 0; i < cl.parent.size(); ++i) cl.parent[i] = i;
    for (size_t oi = 0; oi < porder.size(); ++oi) {
        const Pillar &p = pillars[porder[oi]];
        for (size_t oj = oi + 1; oj < porder.size(); ++oj) {
            const Pillar &q = pillars[porder[oj]];
            if (q.endpoint().x() - p.endpoint().x() >= p.r_start + rmax_pillar + c) break;
            const double d = (V2(p.endpoint().head<2>()) - V2(q.endpoint().head<2>())).norm();
            if (d < p.r_start + q.r_start + c) cl.unite(porder[oi], porder[oj]);
        }
    }
    {
        std::map<size_t, size_t> sizes;
        for (size_t i = 0; i < pillars.size(); ++i) ++sizes[cl.find(i)];
        for (const auto &[root, n] : sizes) {
            m.clusters += n > 1;
            m.max_cluster = std::max(m.max_cluster, n);
        }
    }

    // ── partners per pillar ──
    std::vector<size_t> partners(pillars.size(), 0);
    for (const Group &g : gs) { ++partners[size_t(g.a)]; ++partners[size_t(g.b)]; }
    for (size_t n : partners) {
        m.max_partners = std::max(m.max_partners, n);
        ++m.partners_histogram[std::min(n, m.partners_histogram.size() - 1)];
    }

    // ── truss vs truss, plan view ──
    // Every pair of groups: groups leaving one cluster from pillars far apart
    // along it share it however far apart their boxes are.
    auto xy = [&pillars](long id) { return V2(pillars[size_t(id)].endpoint().head<2>()); };
    std::set<size_t> in_plan;
    for (size_t gi = 0; gi < gs.size(); ++gi) {
        const Group &g = gs[gi];
        for (size_t hi = gi + 1; hi < gs.size(); ++hi) {
            const Group &h = gs[hi];

            // Every end of g against every end of h that shares a pillar or a
            // cluster with it: the angle, each from its own pillar's axis.
            bool shared = false, bad = false;
            for (long ge : {g.a, g.b})
                for (long he : {h.a, h.b}) {
                    if (!cl.same(ge, he)) continue;
                    shared = true;
                    const V2 u = xy(ge == g.a ? g.b : g.a) - xy(ge);
                    const V2 v = xy(he == h.a ? h.b : h.a) - xy(he);
                    const double lu = u.norm(), lv = v.norm();
                    if (lu > 1e-9 && lv > 1e-9 &&
                        std::clamp(u.dot(v) / (lu * lv), -1., 1.) > cos_min)
                        bad = true;
                }

            const bool relaxed = g.relaxed || h.relaxed;
            if (shared) {
                (relaxed ? m.shared_angle_relaxed : m.shared_angle) += bad;
            } else {
                bad = seg_seg_dist2d(g.pa, g.pb, h.pa, h.pb) < g.r + h.r + c;
                (relaxed ? m.cross_truss_relaxed : m.cross_truss) += bad;
            }
            if (bad && !relaxed) { in_plan.insert(gi); in_plan.insert(hi); }
        }
    }
    m.groups_in_plan_violation = in_plan.size();

    // ── truss vs third pillar axis ──
    const double preach = rmax_bar + rmax_pillar + c;
    std::set<size_t> in_obstacle;
    for (size_t gi = 0; gi < gs.size(); ++gi) {
        const Group &g = gs[gi];
        auto it = std::lower_bound(porder.begin(), porder.end(), g.minx - preach,
                                   [&pillars](size_t idx, double x) {
                                       return pillars[idx].endpoint().x() < x;
                                   });
        for (; it != porder.end(); ++it) {
            const Pillar &p = pillars[*it];
            if (p.endpoint().x() > g.maxx + preach) break;
            if (cl.same(long(*it), g.a) || cl.same(long(*it), g.b)) continue;
            const V2 pc = p.endpoint().head<2>();
            if (pc.y() > g.maxy + preach || pc.y() < g.miny - preach) continue;
            if (point_seg_dist2d(pc, g.pa, g.pb) < g.r + p.r_start + c) {
                if (g.relaxed) { ++m.obstacle_pillar_relaxed; continue; }
                ++m.obstacle_pillar;
                m.obstacle_pillar_offground += std::abs(p.endpoint().z() - ground_z) >= 1e-6;
                in_obstacle.insert(gi);
            }
        }
    }
    m.groups_in_obstacle_violation = in_obstacle.size();

    // ── isolated tall pillars ──
    for (const Pillar &p : pillars) {
        if (p.frozen || !(p.height > BracingMetrics::TALL_PILLAR_MM)) continue;
        ++m.tall;
        if (p.links != 0) continue;
        ++m.isolated_tall;
        m.isolated_tall_ground += std::abs(p.endpoint().z() - ground_z) < 1e-6;
    }

    return m;
}

std::string BracingMetrics::to_json() const
{
    std::ostringstream o;
    o << "{\"heads_valid\":" << heads_valid << ",\"pillars\":" << pillars
      << ",\"frozen\":" << frozen << ",\"aux_pillars\":" << aux_pillars
      << ",\"truss_bars\":" << truss_bars << ",\"truss_groups\":" << truss_groups
      << ",\"orphan_truss\":" << orphan_truss << ",\"head_bars\":" << head_bars
      << ",\"prop_bars\":" << prop_bars << ",\"route_bars\":" << route_bars
      << ",\"route_diff\":" << route_diff
      << ",\"kind_unset\":" << kind_unset << ",\"kind_disagrees\":" << kind_disagrees
      << ",\"clusters\":" << clusters << ",\"max_cluster\":" << max_cluster
      << ",\"relaxed_groups\":" << relaxed_groups
      << ",\"fallback_groups\":" << fallback_groups
      << ",\"cross_truss\":" << cross_truss
      << ",\"shared_angle\":" << shared_angle
      << ",\"cross_truss_relaxed\":" << cross_truss_relaxed
      << ",\"shared_angle_relaxed\":" << shared_angle_relaxed
      << ",\"obstacle_pillar\":" << obstacle_pillar
      << ",\"obstacle_pillar_offground\":" << obstacle_pillar_offground
      << ",\"obstacle_pillar_relaxed\":" << obstacle_pillar_relaxed
      << ",\"groups_in_plan_violation\":" << groups_in_plan_violation
      << ",\"groups_in_obstacle_violation\":" << groups_in_obstacle_violation
      << ",\"tall\":" << tall << ",\"isolated_tall\":" << isolated_tall
      << ",\"isolated_tall_ground\":" << isolated_tall_ground
      << ",\"max_partners\":" << max_partners << ",\"partners_histogram\":[";
    for (size_t i = 0; i < partners_histogram.size(); ++i)
        o << (i ? "," : "") << partners_histogram[i];
    o << "]}";
    return o.str();
}

BuilderDigests builder_digests(const SupportTreeBuilder &bld)
{
    BuilderDigests d;

    Fnv fh;
    for (const Head &h : bld.heads()) {
        fh.pod(h.id); fh.pod(h.is_valid()); fh.v3(h.pos); fh.v3(h.dir);
        fh.pod(h.r_back_mm); fh.pod(h.r_pin_mm); fh.pod(h.width_mm);
        fh.pod(h.pillar_id); fh.pod(h.bridge_id);
    }
    d.heads = fh.hex();

    Fnv fp;
    for (const Pillar &p : bld.pillars()) {
        fp.pod(p.id); fp.v3(p.endpt); fp.pod(p.height); fp.pod(p.r_start);
        fp.pod(p.r_end); fp.pod(p.links); fp.pod(p.bridges); fp.pod(p.props_for);
    }
    d.pillars = fp.hex();

    Fnv fb, fc, fd;
    for (const Bridge &b : bld.bridges()) hash_bridge(fb, b);
    for (const Bridge &b : bld.crossbridges()) hash_bridge(fc, b);
    for (const DiffBridge &b : bld.diffbridges()) { hash_bridge(fd, b); fd.pod(b.end_r); }
    d.bridges = fb.hex(); d.crossbridges = fc.hex(); d.diffbridges = fd.hex();

    const indexed_triangle_set &its = bld.retrieve_mesh(MeshType::Support);
    Fnv fm;
    for (const auto &v : its.vertices) fm.bytes(v.data(), sizeof(float) * 3);
    for (const auto &f : its.indices) fm.bytes(f.data(), sizeof(int) * 3);
    d.mesh          = fm.hex();
    d.mesh_vertices = its.vertices.size();
    d.mesh_faces    = its.indices.size();

    return d;
}

}} // namespace Slic3r::sla

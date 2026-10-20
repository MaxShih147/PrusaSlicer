#include "RoutePlan.hpp"

#include <cassert>
#include <iomanip>
#include <sstream>
#include <string>

#include <boost/log/trivial.hpp>

namespace Slic3r { namespace sla {

using Op = BuildStep::Op;

long BuildRecipe::add_pillar(long head_id, double length)
{
    BuildStep s{Op::head_pillar};
    s.head = head_id;
    s.len  = length;
    push(s);
    return LOCAL + m_pillars++;
}

long BuildRecipe::add_pillar(const Vec3d &endpt, double height, double r_start, double r_end)
{
    BuildStep s{Op::pillar};
    s.a   = endpt;
    s.len = height;
    s.r   = r_start;
    s.r2  = r_end;
    push(s);
    return LOCAL + m_pillars++;
}

void BuildRecipe::add_pillar_base(long pillar_id, double height, double radius)
{
    BuildStep s{Op::pillar_base};
    s.pillar = pillar_id;
    s.len    = height;
    s.r      = radius;
    push(s);
}

void BuildRecipe::add_junction(const Vec3d &pos, double r)
{
    BuildStep s{Op::junction};
    s.a = pos;
    s.r = r;
    push(s);
    ++m_junctions;
}

void BuildRecipe::own_junctions_from(size_t first, long pillar_id)
{
    BuildStep s{Op::own_junctions};
    s.ref    = long(first);
    s.pillar = pillar_id;
    push(s);
}

Bridge BuildRecipe::add_bridge(const Vec3d &st, const Vec3d &e, double r, BridgeKind kind,
                               long pillar_id)
{
    BuildStep s{Op::bridge};
    s.a      = st;
    s.b      = e;
    s.r      = r;
    s.kind   = kind;
    s.pillar = pillar_id;
    push(s);

    Bridge br{st, e, r};
    br.id      = LOCAL + m_bridges++;
    br.kind    = kind;
    br.owner_b = pillar_id;
    return br;
}

Bridge BuildRecipe::add_bridge_between(long pillar_id, const Vec3d &st, const Vec3d &e, double r,
                                       BridgeKind kind, long from_pillar)
{
    BuildStep s{Op::bridge_between};
    s.pillar = pillar_id;
    s.a      = st;
    s.b      = e;
    s.r      = r;
    s.kind   = kind;
    s.ref    = from_pillar;
    push(s);

    Bridge br{st, e, r};
    br.id      = LOCAL + m_bridges++;
    br.kind    = kind;
    br.owner_a = from_pillar;
    br.owner_b = pillar_id;
    return br;
}

void BuildRecipe::add_bridge_to_pillar(long head_id, const Vec3d &endp, long pillar_id)
{
    BuildStep s{Op::head_bridge};
    s.head   = head_id;
    s.b      = endp;
    s.pillar = pillar_id;
    push(s);
    ++m_bridges;
}

DiffBridge BuildRecipe::add_diffbridge(const DiffBridge &br)
{
    BuildStep s{Op::diffbridge};
    s.a  = br.startp;
    s.b  = br.endp;
    s.r  = br.r;
    s.r2 = br.end_r;
    push(s);

    DiffBridge out{br.startp, br.endp, br.r, br.end_r};
    out.id   = LOCAL + m_diffbridges++;
    out.kind = BridgeKind::route;
    return out;
}

void BuildRecipe::own_bridge(long bridge_id, long pillar_id)
{
    BuildStep s{Op::own_bridge};
    s.ref    = bridge_id;
    s.pillar = pillar_id;
    push(s);
}

void BuildRecipe::own_diffbridge(long bridge_id, long pillar_id)
{
    BuildStep s{Op::own_diffbridge};
    s.ref    = bridge_id;
    s.pillar = pillar_id;
    push(s);
}

void BuildRecipe::set_head_bridge(long head_id, long bridge_id)
{
    BuildStep s{Op::head_bridge_id};
    s.head = head_id;
    s.ref  = bridge_id;
    push(s);
}

void BuildRecipe::set_head_diffbridge(long head_id, long bridge_id)
{
    BuildStep s{Op::head_diffbridge_id};
    s.head = head_id;
    s.ref  = bridge_id;
    push(s);
}

void BuildRecipe::set_head_pillar(long head_id, long pillar_id)
{
    BuildStep s{Op::head_pillar_id};
    s.head   = head_id;
    s.pillar = pillar_id;
    push(s);
}

void BuildRecipe::add_anchor(double r_back, double r_pin, double width, double penetration,
                             const Vec3d &dir, const Vec3d &pos)
{
    BuildStep s{Op::anchor};
    s.r   = r_back;
    s.r2  = r_pin;
    s.len = width;
    s.pen = penetration;
    s.b   = dir;
    s.a   = pos;
    push(s);
}

void BuildRecipe::increment_bridges(long pillar_id)
{
    BuildStep s{Op::increment_bridges};
    s.pillar = pillar_id;
    push(s);
}

RecipeIds replay(SupportTreeBuilder &builder, const BuildRecipe &recipe)
{
    RecipeIds ids;

    // A recipe-local id to the builder's; any other id is the builder's already.
    auto real = [](const std::vector<long> &made, long id) {
        if (!BuildRecipe::is_local(id))
            return id;
        assert(size_t(id - BuildRecipe::LOCAL) < made.size());
        return made[size_t(id - BuildRecipe::LOCAL)];
    };
    auto P = [&](long id) { return real(ids.pillars, id); };
    auto B = [&](long id) { return real(ids.bridges, id); };
    auto D = [&](long id) { return real(ids.diffbridges, id); };

    for (const BuildStep &s : recipe.steps()) {
        switch (s.op) {
        case Op::head_pillar:
            ids.pillars.push_back(builder.add_pillar(s.head, s.len));
            break;
        case Op::pillar:
            ids.pillars.push_back(builder.add_pillar(s.a, s.len, s.r, s.r2));
            break;
        case Op::pillar_base:
            builder.add_pillar_base(P(s.pillar), s.len, s.r);
            break;
        case Op::junction:
            ids.junctions.push_back(builder.add_junction(s.a, s.r).id);
            break;
        case Op::own_junctions: {
            // From the first junction the recipe made at or after that point;
            // none made since means none to own, as on the builder.
            const size_t k = size_t(s.ref - BuildRecipe::LOCAL);
            const size_t first = k < ids.junctions.size() ? size_t(ids.junctions[k])
                                                          : builder.junctioncount();
            builder.own_junctions_from(first, P(s.pillar));
            break;
        }
        case Op::bridge:
            ids.bridges.push_back(builder.add_bridge(s.a, s.b, s.r, s.kind, P(s.pillar)).id);
            break;
        case Op::bridge_between:
            ids.bridges.push_back(
                builder.add_bridge_between(P(s.pillar), s.a, s.b, s.r, s.kind, P(s.ref)).id);
            break;
        case Op::head_bridge:
            ids.bridges.push_back(builder.add_bridge_to_pillar(s.head, s.b, P(s.pillar)).id);
            break;
        case Op::diffbridge:
            ids.diffbridges.push_back(builder.add_diffbridge(DiffBridge{s.a, s.b, s.r, s.r2}).id);
            break;
        case Op::own_bridge:
            builder.own_bridge(B(s.ref), P(s.pillar));
            break;
        case Op::own_diffbridge:
            builder.own_diffbridge(D(s.ref), P(s.pillar));
            break;
        case Op::head_bridge_id:
            builder.head(unsigned(s.head)).bridge_id = B(s.ref);
            break;
        case Op::head_diffbridge_id:
            builder.head(unsigned(s.head)).bridge_id = D(s.ref);
            break;
        case Op::head_pillar_id:
            builder.head(unsigned(s.head)).pillar_id = P(s.pillar);
            break;
        case Op::anchor:
            builder.add_anchor(s.r, s.r2, s.len, s.pen, s.b, s.a);
            break;
        case Op::increment_bridges:
            builder.increment_bridges(builder.pillar(P(s.pillar)));
            break;
        }
    }

    return ids;
}

const char *route_kind_name(RoutePlan::Kind kind)
{
    switch (kind) {
    case RoutePlan::Kind::to_pillar:    return "to_pillar";
    case RoutePlan::Kind::own_pillar:   return "own_pillar";
    case RoutePlan::Kind::ground_route: return "ground_route";
    case RoutePlan::Kind::model_anchor: return "model_anchor";
    }
    return "?";
}

const char *route_level_name(RouteLevel level)
{
    switch (level) {
    case RouteLevel::L0:   return "L0";
    case RouteLevel::L1:   return "L1";
    case RouteLevel::L2:   return "L2";
    case RouteLevel::none: return "none";
    }
    return "?";
}

std::vector<clearance::Prim> route_prims(const BuildRecipe &recipe, const Head &head)
{
    using namespace clearance;

    std::vector<Prim> out;
    long pillars = 0;                   // pillars the recipe made so far
    std::vector<size_t> bars, diffbars; // the prim of the recipe's k-th bridge, diffbridge

    auto add = [&out, &head](const Capsule &cap, PrimKind kind, long a, long b) {
        Prim p;
        p.cap      = cap;
        p.kind     = kind;
        p.head_id  = head.id;
        p.pillar_a = a;
        p.pillar_b = b;
        out.emplace_back(p);
        return out.size() - 1;
    };
    auto bar_kind = [](BridgeKind k) {
        return k == BridgeKind::head ? PrimKind::head_bar :
               k == BridgeKind::prop ? PrimKind::prop_bar : PrimKind::route_bar;
    };
    // The prim made for a bridge id the recipe handed out.
    auto made_for = [](const std::vector<size_t> &made, long id) {
        assert(BuildRecipe::is_local(id) && size_t(id - BuildRecipe::LOCAL) < made.size());
        return made[size_t(id - BuildRecipe::LOCAL)];
    };

    for (const BuildStep &s : recipe.steps()) {
        switch (s.op) {
        case Op::head_pillar: {
            // As SupportTreeBuilder::add_pillar(head, length) makes it.
            assert(s.head == head.id);
            const Vec3d foot = head.junction_point() - Vec3d{0., 0., s.len};
            add(pillar_capsule(foot, s.len, head.r_back_mm, head.r_back_mm), PrimKind::pillar,
                BuildRecipe::LOCAL + pillars++, -1);
            break;
        }
        case Op::pillar:
            add(pillar_capsule(s.a, s.len, s.r, s.r2), PrimKind::pillar,
                BuildRecipe::LOCAL + pillars++, -1);
            break;
        case Op::bridge:
            bars.push_back(add(bar_capsule(s.a, s.b, s.r), bar_kind(s.kind), -1, s.pillar));
            break;
        case Op::bridge_between:
            bars.push_back(add(bar_capsule(s.a, s.b, s.r), bar_kind(s.kind), s.ref, s.pillar));
            break;
        case Op::head_bridge:
            // As SupportTreeBuilder::add_bridge(head, endp, pillar) makes it.
            assert(s.head == head.id);
            bars.push_back(add(bar_capsule(head.junction_point(), s.b, head.r_back_mm),
                               PrimKind::head_bar, -1, s.pillar));
            break;
        case Op::diffbridge:
            diffbars.push_back(add(diffbridge_capsule(s.a, s.b, s.r, s.r2), PrimKind::route_bar,
                                   -1, -1));
            break;
        case Op::own_bridge:
            out[made_for(bars, s.ref)].pillar_b = s.pillar;
            break;
        case Op::own_diffbridge:
            out[made_for(diffbars, s.ref)].pillar_b = s.pillar;
            break;
        case Op::anchor: {
            // A reverse pinhead, under the pillar the recipe made last.
            const Head anchor(s.r, s.r2, s.len, s.pen, s.b, s.a);
            add(pinhead_capsule(anchor.pos, anchor.junction_point(), anchor.r_back_mm),
                PrimKind::pinhead, pillars > 0 ? BuildRecipe::LOCAL + pillars - 1 : -1, -1);
            break;
        }
        case Op::pillar_base:
        case Op::junction:
        case Op::own_junctions:
        case Op::head_bridge_id:
        case Op::head_diffbridge_id:
        case Op::head_pillar_id:
        case Op::increment_bridges:
            break; // no element of its own
        }
    }

    return out;
}

void to_builder_ids(std::vector<clearance::Prim> &prims, const RecipeIds &ids)
{
    for (clearance::Prim &p : prims) {
        p.pillar_a = ids.pillar(p.pillar_a);
        p.pillar_b = ids.pillar(p.pillar_b);
    }
}

bool can_bridge_to(const SupportTreeBuilder &builder, const Pillar &pillar, const Head &head,
                   unsigned max_bridges)
{
    return builder.bridgecount(pillar) < max_bridges && pillar.r_start >= head.r_back_mm;
}

bool hard_rules_ok(const SupportTreeBuilder &builder, const RoutePlan &plan, const Head &head,
                   unsigned max_bridges)
{
    return plan.kind != RoutePlan::Kind::to_pillar ||
           can_bridge_to(builder, builder.pillar(plan.target_pillar), head, max_bridges);
}

RouteChoice select_route(const RouteList                 &cands,
                         const clearance::ClearanceIndex &index,
                         const clearance::PillarAxisFn   &axis_of,
                         const HardRuleFn                &hard_ok,
                         const RecomputeFn               &recompute)
{
    using clearance::SAFETY_CLEARANCE;

    // The hard rules first, then the clearance: a candidate the rules turn
    // down is never judged, at any level.
    auto first_clear = [&](const RouteList &list, double c) -> const RoutePlan * {
        for (const RoutePlan *p : list)
            if (hard_ok(*p) && index.violations(p->prims, c, axis_of).empty())
                return p;
        return nullptr;
    };

    RouteChoice out;
    out.level = RouteLevel::L0;
    if ((out.plan = first_clear(cands, SAFETY_CLEARANCE)))
        return out;

    // Once, on what is there now: the routes committed since the head was
    // planned may have made a pillar it can reach, and a fallback the first
    // plan left out is planned now. From here on only the new list counts.
    const RouteList again = recompute();
    out.recomputed = true;
    if ((out.plan = first_clear(again, SAFETY_CLEARANCE)))
        return out;

    out.level = RouteLevel::L1;
    out.plan  = first_clear(again, 0.);

    if (!out.plan) {
        out.level = RouteLevel::L2;
        for (const RoutePlan *p : again)
            if (hard_ok(*p)) {
                out.plan = p;
                break;
            }
    }

    if (!out.plan) {
        out.level = RouteLevel::none;
        return out;
    }

    out.contacts = index.violations(out.plan->prims, SAFETY_CLEARANCE, axis_of);
    return out;
}

std::vector<clearance::Contact> centroid_contacts(const std::vector<clearance::Prim> &route,
                                                  const clearance::Prim             &grower,
                                                  const clearance::ClearanceIndex   &index,
                                                  const clearance::PillarAxisFn     &axis_of,
                                                  size_t                            &native)
{
    using namespace clearance;

    std::vector<Contact> out;
    for (const Contact &ct : index.violations(route, SAFETY_CLEARANCE, axis_of)) {
        const Prim &with = index.prims()[ct.with_index];
        // A pillar against a head the head it grew from is itself that close
        // to, bare (no clipping): the spec's "原生過密".
        if (route[ct.cand].kind == PrimKind::pillar && with.kind == PrimKind::pinhead &&
            capsule_dist(grower.cap, with.cap) < grower.cap.r + with.cap.r + SAFETY_CLEARANCE) {
            ++native;
            continue;
        }
        out.emplace_back(ct);
    }
    return out;
}

namespace {

// " at (x,y,z) (x,y,z) ...": where each contact is, on the element judged.
std::string contact_points(const std::vector<clearance::Contact> &contacts)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(3) << " at";
    for (const clearance::Contact &ct : contacts)
        os << " (" << ct.p_new.x() << "," << ct.p_new.y() << "," << ct.p_new.z() << ")";
    return os.str();
}

} // namespace

void log_route_record(const RouteRecord &rec)
{
    std::ostringstream os;
    os << "head=" << rec.head << " kind=" << route_kind_name(rec.kind)
       << " contacts=" << rec.contacts.size() << contact_points(rec.contacts);

    if (rec.level == RouteLevel::L2)
        BOOST_LOG_TRIVIAL(warning) << "WARNING: fallback_intersection_retained " << os.str();
    else
        BOOST_LOG_TRIVIAL(info) << "clearance_relaxed " << os.str();
}

void log_centroid_record(const CentroidRecord &rec)
{
    const clearance::Contact &ct = rec.contact;
    BOOST_LOG_TRIVIAL(info) << "centroid_pillar_clearance_recorded pillar=" << rec.pillar
                            << " head=" << ct.with_head << std::fixed << std::setprecision(3)
                            << " dist=" << ct.dist << " required=" << ct.required
                            << contact_points({ct}) << " with=" << clearance::kind_name(ct.with);
}

void report_route_clearance(const ClearanceReport &report, const char *tree_name)
{
    BOOST_LOG_TRIVIAL(info) << tree_name << ": head route clearance: L1 " << report.l1
                            << ", L2 " << report.l2 << ", none " << report.none
                            << ", centre pillar contacts recorded " << report.centroid.size()
                            << ", support point spacing " << report.native;
}

}} // namespace Slic3r::sla

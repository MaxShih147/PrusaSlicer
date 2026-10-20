// Choosing a head's route (openspec change sla-head-bridge-clearance, task
// 4.7): select_route() driven by made-up candidates against an index built by
// hand, the hard rules it must never relax, the order of pillar candidates,
// what a route's recipe makes as capsules, and the contacts of a centre pillar.
//
// The committed element in most cases is one head bar of head 1, along x at
// z = 10. Head 2's candidates are bars along y crossing over it at a height
// chosen for the distance wanted: radii 0.5 and 0.5, so 1.2 keeps the
// clearance (L0) and 1.0 only does not touch (L1).

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <algorithm>
#include <vector>

#include "libslic3r/SLA/BridgeClearance.hpp"
#include "libslic3r/SLA/RoutePlan.hpp"
#include "libslic3r/SLA/SupportTreeBuilder.hpp"

using namespace Slic3r;
using namespace Slic3r::sla;
using namespace Slic3r::sla::clearance;
using Catch::Approx;

namespace {

constexpr long HEAD = 2; // the head being routed

Prim bar(const Vec3d &a, const Vec3d &b, long head, long pa = -1, long pb = -1)
{
    Prim p;
    p.cap      = bar_capsule(a, b, 0.5);
    p.kind     = PrimKind::head_bar;
    p.head_id  = head;
    p.pillar_a = pa;
    p.pillar_b = pb;
    return p;
}

Prim pinhead(const Vec3d &pos, const Vec3d &junction, long head)
{
    Prim p;
    p.cap     = pinhead_capsule(pos, junction, 0.5);
    p.kind    = PrimKind::pinhead;
    p.head_id = head;
    return p;
}

// Head 1's bar, the one in the way.
ClearanceIndex committed()
{
    ClearanceIndex index;
    index.insert(bar({-5., 0., 10.}, {5., 0., 10.}, 1));
    return index;
}

// A candidate of head 2 whose bar passes `dist` mm over head 1's.
RoutePlan crossing(double dist, RoutePlan::Kind kind = RoutePlan::Kind::own_pillar,
                   long target = SupportTreeNode::ID_UNSET)
{
    RoutePlan p;
    p.kind          = kind;
    p.target_pillar = target;
    p.prims.push_back(bar({0., -5., 10. + dist}, {0., 5., 10. + dist}, HEAD));
    return p;
}

// No shared pillar in these cases.
JointAxis no_axis(long) { return {}; }

bool any(const RoutePlan &) { return true; }

// Counts the calls and hands back `list`.
struct Recompute
{
    RouteList list;
    int       calls = 0;

    RecomputeFn fn()
    {
        return [this] {
            ++calls;
            return list;
        };
    }
};

} // namespace

TEST_CASE("L0 takes the first candidate that keeps the clearance", "[head_route_select]")
{
    const ClearanceIndex index = committed();
    const RoutePlan      near  = crossing(1.0), clear = crossing(2.0);
    Recompute            again;

    const RouteChoice c = select_route({&near, &clear}, index, no_axis, any, again.fn());

    REQUIRE(c.plan == &clear);
    REQUIRE(c.level == RouteLevel::L0);
    REQUIRE_FALSE(c.recomputed);
    REQUIRE(c.contacts.empty());
    REQUIRE(again.calls == 0);
}

TEST_CASE("a head plans again once, and sees what was built since", "[head_route_select]")
{
    // Every first candidate is too close; planned again, the head can reach
    // a pillar a head before it made, and that route keeps the clearance.
    const ClearanceIndex index    = committed();
    const RoutePlan      near     = crossing(1.0);
    const RoutePlan      near_too = crossing(1.1);
    const RoutePlan      new_pillar = crossing(3.0, RoutePlan::Kind::to_pillar, 7);
    Recompute            again{{&near_too, &new_pillar}};

    const RouteChoice c = select_route({&near}, index, no_axis, any, again.fn());

    REQUIRE(again.calls == 1);
    REQUIRE(c.recomputed);
    REQUIRE(c.plan == &new_pillar);
    REQUIRE(c.level == RouteLevel::L0);
    REQUIRE(c.contacts.empty());
}

TEST_CASE("L1 when only the clearance is missing", "[head_route_select]")
{
    // d = 1.1 against r1 + r2 = 1.0: short of L0's 1.2, clear of touching.
    // The first list has such a candidate too; the one taken must come from
    // the list planned again.
    const ClearanceIndex index     = committed();
    const RoutePlan      first     = crossing(1.1);
    const RoutePlan      touches   = crossing(0.8), close = crossing(1.15);
    Recompute            again{{&touches, &close}};

    const RouteChoice c = select_route({&first}, index, no_axis, any, again.fn());

    REQUIRE(again.calls == 1);
    REQUIRE(c.recomputed);
    REQUIRE(c.plan == &close);
    REQUIRE(c.level == RouteLevel::L1);

    // What the route keeps short of L0, for the log.
    REQUIRE(c.contacts.size() == 1);
    REQUIRE(c.contacts[0].dist == Approx(1.15));
    REQUIRE(c.contacts[0].required == Approx(1.0 + SAFETY_CLEARANCE));
    REQUIRE(c.contacts[0].with == PrimKind::head_bar);
    REQUIRE(c.contacts[0].with_head == 1);
    REQUIRE(c.contacts[0].p_new.isApprox(Vec3d{0., 0., 11.15}));
}

TEST_CASE("L2 keeps the first candidate of the new list the rules allow", "[head_route_select]")
{
    // Nothing clears even touching. The candidate kept comes from the list
    // planned again, not the first one, and the hard rules still hold.
    const ClearanceIndex index   = committed();
    const RoutePlan      old     = crossing(0.5);
    const RoutePlan      blocked = crossing(5.0, RoutePlan::Kind::to_pillar, 3);
    const RoutePlan      first   = crossing(0.8), second = crossing(0.9);
    Recompute            again{{&blocked, &first, &second}};

    auto hard_ok = [&blocked](const RoutePlan &p) { return &p != &blocked; };

    const RouteChoice c = select_route({&old}, index, no_axis, hard_ok, again.fn());

    REQUIRE(again.calls == 1);
    REQUIRE(c.plan == &first);
    REQUIRE(c.level == RouteLevel::L2);
    REQUIRE(c.contacts.size() == 1);
    REQUIRE(c.contacts[0].dist == Approx(0.8));
    REQUIRE(c.contacts[0].dist < 1.0); // touching: kept anyway, recorded
}

TEST_CASE("none when no candidate passes the hard rules", "[head_route_select]")
{
    const ClearanceIndex index = committed();
    const RoutePlan      clear = crossing(5.0, RoutePlan::Kind::to_pillar, 3);
    Recompute            again{{&clear}};

    auto never = [](const RoutePlan &) { return false; };

    const RouteChoice c = select_route({&clear}, index, no_axis, never, again.fn());

    REQUIRE(again.calls == 1);
    REQUIRE(c.plan == nullptr);
    REQUIRE(c.level == RouteLevel::none);
    REQUIRE(c.contacts.empty());

    // With no candidate at all, the same.
    Recompute nothing;
    const RouteChoice e = select_route({}, index, no_axis, any, nothing.fn());
    REQUIRE(nothing.calls == 1);
    REQUIRE(e.plan == nullptr);
    REQUIRE(e.level == RouteLevel::none);
}

TEST_CASE("no level relaxes how thick a pillar or how full", "[head_route_select]")
{
    // Head 0 is 0.75 mm at the back. Pillar 0 is thinner, pillar 1 as thick,
    // pillar 2 as thick but full.
    const unsigned     cap = 3;
    SupportTreeBuilder b;
    const Head        &head = b.add_head(0, 0.75, 0.2, 1., 0.5, DOWN, Vec3d{0., 0., 20.});
    const long thin  = b.add_pillar(Vec3d{3., 0., 0.}, 10., 0.3);
    const long thick = b.add_pillar(Vec3d{0., 3., 0.}, 10., 0.75);
    const long full  = b.add_pillar(Vec3d{-3., 0., 0.}, 10., 0.75);
    for (unsigned i = 0; i < cap; ++i)
        b.increment_bridges(b.pillar(full));

    auto hard_ok = [&](const RoutePlan &p) { return hard_rules_ok(b, p, head, cap); };

    const ClearanceIndex index = committed();
    const RoutePlan to_thin  = crossing(5.0, RoutePlan::Kind::to_pillar, thin);
    const RoutePlan to_thick = crossing(5.0, RoutePlan::Kind::to_pillar, thick);
    const RoutePlan to_full  = crossing(5.0, RoutePlan::Kind::to_pillar, full);
    const RoutePlan own      = crossing(0.8); // touching, but its own pillar

    REQUIRE_FALSE(hard_ok(to_thin));
    REQUIRE(hard_ok(to_thick));
    REQUIRE_FALSE(hard_ok(to_full));
    REQUIRE(hard_ok(own));

    SECTION("L0 passes over a thin pillar and a full one, clear as they are") {
        Recompute again;
        const RouteChoice c = select_route({&to_thin, &to_full, &to_thick}, index, no_axis,
                                           hard_ok, again.fn());
        REQUIRE(c.plan == &to_thick);
        REQUIRE(c.level == RouteLevel::L0);
        REQUIRE(again.calls == 0);
    }

    SECTION("L2 keeps its own pillar, touching, rather than a thin or full one") {
        Recompute again{{&to_thin, &to_full, &own}};
        const RouteChoice c = select_route({&to_thin, &to_full, &own}, index, no_axis,
                                           hard_ok, again.fn());
        REQUIRE(c.plan == &own);
        REQUIRE(c.level == RouteLevel::L2);
    }

    SECTION("a thin pillar is never kept, even as the only way") {
        Recompute again{{&to_thin}};
        const RouteChoice c = select_route({&to_thin}, index, no_axis, hard_ok, again.fn());
        REQUIRE(c.plan == nullptr);
        REQUIRE(c.level == RouteLevel::none);
    }

    SECTION("one bridge short of the cap takes one more") {
        const long room = b.add_pillar(Vec3d{0., -3., 0.}, 10., 0.75);
        for (unsigned i = 0; i + 1 < cap; ++i)
            b.increment_bridges(b.pillar(room));
        const RoutePlan to_room = crossing(5.0, RoutePlan::Kind::to_pillar, room);
        REQUIRE(hard_ok(to_room));
        b.increment_bridges(b.pillar(room));
        REQUIRE_FALSE(hard_ok(to_room));
    }
}

TEST_CASE("candidates at the same distance go by pillar id", "[head_route_select]")
{
    std::vector<NearPillar> near{{4., 12}, {4., 7}, {9., 1}, {1., 20}};

    std::vector<NearPillar> sorted = near;
    std::sort(sorted.begin(), sorted.end(), nearer);
    REQUIRE(sorted[0].id == 20);
    REQUIRE(sorted[1].id == 7);
    REQUIRE(sorted[2].id == 12);
    REQUIRE(sorted[3].id == 1);

    // However they come in.
    std::reverse(near.begin(), near.end());
    std::partial_sort(near.begin(), near.begin() + 3, near.end(), nearer);
    REQUIRE(near[1].id == 7);
    REQUIRE(near[2].id == 12);
}

TEST_CASE("what a route's recipe makes, as capsules", "[head_route_select]")
{
    const Head head = [] {
        Head h(0.5, 0.2, 1., 0.5, DOWN, Vec3d{0., 0., 20.});
        h.id = HEAD;
        return h;
    }();
    const Vec3d jp = head.junction_point();

    SECTION("a bar to a pillar, from a short pillar of the head's own") {
        BuildRecipe r;
        const long   own   = r.add_pillar(head.id, 2.);
        const size_t first = r.junctioncount();
        const Vec3d  start = jp - Vec3d{0., 0., 2.}, end{3., 0., 15.};
        r.add_junction(start, 0.5);
        r.own_junctions_from(first, own);
        r.add_bridge_between(5, start, end, 0.5, BridgeKind::head, own);
        r.increment_bridges(5);

        std::vector<Prim> prims = route_prims(r, head);
        REQUIRE(prims.size() == 2);

        REQUIRE(prims[0].kind == PrimKind::pillar);
        REQUIRE(prims[0].pillar_a == own);
        REQUIRE(prims[0].cap.a.isApprox(start));
        REQUIRE(prims[0].cap.b.isApprox(jp));
        REQUIRE(prims[0].cap.r == 0.5);

        REQUIRE(prims[1].kind == PrimKind::head_bar);
        REQUIRE(prims[1].pillar_a == own);
        REQUIRE(prims[1].pillar_b == 5);
        REQUIRE(prims[1].cap.a.isApprox(start));
        REQUIRE(prims[1].cap.b.isApprox(end));

        for (const Prim &p : prims)
            REQUIRE(p.head_id == HEAD);

        RecipeIds ids;
        ids.pillars = {9};
        to_builder_ids(prims, ids);
        REQUIRE(prims[0].pillar_a == 9);
        REQUIRE(prims[1].pillar_a == 9);
        REQUIRE(prims[1].pillar_b == 5);
    }

    SECTION("a bar straight from the junction") {
        BuildRecipe r;
        r.add_bridge_to_pillar(head.id, Vec3d{4., 0., 16.}, 3);

        const std::vector<Prim> prims = route_prims(r, head);
        REQUIRE(prims.size() == 1);
        REQUIRE(prims[0].kind == PrimKind::head_bar);
        REQUIRE(prims[0].cap.a.isApprox(jp));
        REQUIRE(prims[0].cap.r == head.r_back_mm);
        REQUIRE(prims[0].pillar_b == 3);
    }

    SECTION("a route around the model, its pillar named afterwards") {
        BuildRecipe      r;
        const DiffBridge wide = r.add_diffbridge(DiffBridge{jp, Vec3d{2., 0., 17.}, 0.25, 0.5});
        const Bridge     out  = r.add_bridge(Vec3d{2., 0., 17.}, Vec3d{4., 0., 15.}, 0.5,
                                             BridgeKind::route);
        const long       pid  = r.add_pillar(Vec3d{4., 0., 0.}, 15., 0.5, 0.5);
        r.own_bridge(out.id, pid);
        r.own_diffbridge(wide.id, pid);

        const std::vector<Prim> prims = route_prims(r, head);
        REQUIRE(prims.size() == 3);
        REQUIRE(prims[0].kind == PrimKind::route_bar);
        REQUIRE(prims[0].cap.r == 0.5); // the larger end of the widening bar
        REQUIRE(prims[0].pillar_b == pid);
        REQUIRE(prims[1].kind == PrimKind::route_bar);
        REQUIRE(prims[1].pillar_b == pid);
        REQUIRE(prims[2].kind == PrimKind::pillar);
        REQUIRE(prims[2].pillar_a == pid);
        REQUIRE(prims[2].cap.b.isApprox(Vec3d{4., 0., 15.}));
    }

    SECTION("a pillar down to an anchor on the model") {
        BuildRecipe r;
        const long  pid = r.add_pillar(head.id, 5.);
        const Vec3d dir{0., 0., 1.}, pos{0., 0., 10.};
        r.add_anchor(0.5, 0.2, 1., 0.3, dir, pos);

        const std::vector<Prim> prims = route_prims(r, head);
        REQUIRE(prims.size() == 2);
        REQUIRE(prims[1].kind == PrimKind::pinhead);
        REQUIRE(prims[1].pillar_a == pid);
        REQUIRE(prims[1].head_id == HEAD);
        REQUIRE(prims[1].cap.a.isApprox(pos));
        const Head anchor(0.5, 0.2, 1., 0.3, dir, pos);
        REQUIRE(prims[1].cap.b.isApprox(anchor.junction_point()));
    }
}

TEST_CASE("a centre pillar's contacts with the heads", "[head_route_select]")
{
    // Head 1 grows the centre pillar straight down. Head 2 is lower, 1.16 mm
    // beside the pillar, and well clear of head 1 itself: recorded. Head 3
    // is 0.9 mm beside head 1 already: support point spacing, only counted.
    ClearanceIndex index;
    const size_t grower = index.insert(pinhead({0., 0., 20.}, {0., 0., 18.6}, 1));
    index.insert(pinhead({1.16, 0., 17.}, {1.16, 0., 15.6}, 2));
    index.insert(pinhead({0.9, 0., 20.}, {0.9, 0., 18.6}, 3));

    Prim pillar;
    pillar.cap      = pillar_capsule({0., 0., 0.}, 18.6, 0.5, 0.5);
    pillar.kind     = PrimKind::pillar;
    pillar.head_id  = 1;
    pillar.pillar_a = BuildRecipe::LOCAL;

    size_t native = 0;
    const std::vector<Contact> contacts =
        centroid_contacts({pillar}, index.prims()[grower], index, no_axis, native);

    REQUIRE(native == 1);
    REQUIRE(contacts.size() == 1);
    REQUIRE(contacts[0].with == PrimKind::pinhead);
    REQUIRE(contacts[0].with_head == 2);
    REQUIRE(contacts[0].dist == Approx(1.16));
    REQUIRE(contacts[0].required == Approx(1.0 + SAFETY_CLEARANCE));
}

TEST_CASE("a head's carrier is named once it is routed", "[head_route_select]")
{
    // A bar ending on pillar 4 passes the top of head 5's pinhead, which
    // sits on that pillar: judged whole until the carrier is known, then
    // clipped around the pillar like any two elements sharing it.
    ClearanceIndex index;
    const size_t   head5 = index.insert(pinhead({0., 0., 12.}, {0., 0., 10.6}, 5));
    const Prim     to4   = bar({4., 0., 14.6}, {0., 0., 10.6}, HEAD, -1, 4);

    auto axis = [](long id) {
        REQUIRE(id == 4);
        return JointAxis{Vec2d::Zero(), 0.5};
    };

    REQUIRE_FALSE(index.violations({to4}, SAFETY_CLEARANCE, axis).empty());
    index.set_carrier(head5, 4);
    REQUIRE(index.prims()[head5].pillar_a == 4);
    REQUIRE(index.violations({to4}, SAFETY_CLEARANCE, axis).empty());
}

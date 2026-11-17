// BridgeClearance (openspec change sla-head-bridge-clearance, task group 2).
//
// The scenarios of spec "幾何元件須以 3D 膠囊體表示", "間距判定須以 3D 軸線段
// 最短距離為準" and "共用柱接點須以動態半徑 R 豁免", the edge cases of the
// clipping, and a property test: the index finds exactly what comparing every
// pair finds.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>
#include <limits>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "libslic3r/SLA/BracingPlanner.hpp"
#include "libslic3r/SLA/BridgeClearance.hpp"
#include "libslic3r/SLA/SupportTreeBuilder.hpp"

using namespace Slic3r;
using namespace Slic3r::sla;
using namespace Slic3r::sla::clearance;
using Catch::Approx;

namespace {

constexpr double LIMIT = 0.5 + 0.5 + SAFETY_CLEARANCE; // two 0.5 mm elements at L0
constexpr double DELTA = 1e-6;
constexpr double INF   = std::numeric_limits<double>::infinity();

Vec3d polar3(double deg, double len, double z = 0.)
{
    return {len * std::cos(deg * PI / 180.), len * std::sin(deg * PI / 180.), z};
}

Prim bar(const Vec3d &a, const Vec3d &b, double r = 0.5, long head = -1, long pa = -1,
         long pb = -1, PrimKind kind = PrimKind::head_bar)
{
    Prim p;
    p.cap = bar_capsule(a, b, r);
    p.kind = kind;
    p.head_id = head;
    p.pillar_a = pa;
    p.pillar_b = pb;
    return p;
}

Prim pillar(long id, double x, double y, double foot, double height, double r = 0.5,
            long head = -1)
{
    Prim p;
    p.cap = pillar_capsule({x, y, foot}, height, r, r);
    p.kind = PrimKind::pillar;
    p.head_id = head;
    p.pillar_a = id;
    return p;
}

Prim pinhead(const Vec3d &pos, const Vec3d &junction, double r_back, long head, long carrier)
{
    Prim p;
    p.cap = pinhead_capsule(pos, junction, r_back);
    p.kind = PrimKind::pinhead;
    p.head_id = head;
    p.pillar_a = carrier;
    return p;
}

// Pillars by id for the joint rule.
struct Axes
{
    std::map<long, JointAxis> m;
    PillarAxisFn fn() const
    {
        return [this](long id) { return m.at(id); };
    }
};

const PillarAxisFn no_axes = [](long) -> JointAxis {
    FAIL("no pillar is shared here");
    return {};
};

std::vector<Contact> judge(const Prim &cand, const std::vector<Prim> &old, double c = SAFETY_CLEARANCE,
                           const PillarAxisFn &axes = no_axes)
{
    ClearanceIndex idx;
    for (const Prim &p : old) idx.insert(p);
    return idx.violations({cand}, c, axes);
}

// Two horizontal bars crossing at right angles in plan view, dz apart there.
// At z = 0, so dz itself is the distance, to the last bit.
std::pair<Prim, Prim> crossing(double dz)
{
    return {bar({-5., 0., 0.}, {5., 0., 0.}), bar({0., -5., dz}, {0., 5., dz})};
}

const Vec3d ORIGIN = Vec3d::Zero();

} // namespace

TEST_CASE("one clearance constant", "[bridge_clearance]")
{
    CHECK(SAFETY_CLEARANCE == 0.2);
    CHECK(bracing::SAFETY_CLEARANCE_MM == SAFETY_CLEARANCE);
    CHECK(bracing::PlannerConfig{}.safety_clearance == SAFETY_CLEARANCE);
}

TEST_CASE("elements as capsules", "[bridge_clearance]")
{
    SECTION("bars keep their radius") {
        const Capsule c = bar_capsule({0, 0, 0}, {1, 2, 3}, 0.4);
        CHECK(c.a == Vec3d(0, 0, 0));
        CHECK(c.b == Vec3d(1, 2, 3));
        CHECK(c.r == 0.4);
    }
    SECTION("a DiffBridge takes the larger radius, at either end") {
        CHECK(diffbridge_capsule({0, 0, 0}, {1, 0, 0}, 0.3, 0.5).r == 0.5);
        CHECK(diffbridge_capsule({0, 0, 0}, {1, 0, 0}, 0.5, 0.3).r == 0.5);
    }
    SECTION("a tapered pillar takes 0.75") {
        const Capsule c = pillar_capsule({1, 2, 3}, 10., 0.5, 0.75);
        CHECK(c.r == 0.75);
        CHECK(c.a == Vec3d(1, 2, 3));
        CHECK(c.b == Vec3d(1, 2, 13));
    }
    SECTION("a pinhead runs from its tip to Head::junction_point(), radius r_back") {
        const Head h(0.4, 0.2, 1.0, 0.2, DOWN, {3., 4., 10.});
        const Capsule c = pinhead_capsule(h.pos, h.junction_point(), h.r_back_mm);
        CHECK(c.a == Vec3d(3, 4, 10));
        // 2 r_pin + width + 2 r_back - penetration - r_back = 1.6 down
        CHECK(c.b.x() == 3.);
        CHECK(c.b.y() == 4.);
        CHECK(c.b.z() == Approx(8.4));
        CHECK(c.r == 0.4);
    }
    SECTION("kind names are the export's") {
        CHECK(std::string(kind_name(PrimKind::head_bar)) == "head_bar");
        CHECK(std::string(kind_name(PrimKind::route_bar)) == "route_bar");
        CHECK(std::string(kind_name(PrimKind::prop_bar)) == "prop_bar");
        CHECK(std::string(kind_name(PrimKind::pillar)) == "pillar");
        CHECK(std::string(kind_name(PrimKind::pinhead)) == "pinhead");
    }
}

TEST_CASE("3D distance decides, not height or plan view", "[bridge_clearance]")
{
    SECTION("crossing bars 1.5 apart in height are clear") {
        auto [x, y] = crossing(1.5);
        CHECK(capsule_dist(x.cap, y.cap) == Approx(1.5));
        CHECK(judge(x, {y}).empty());
    }
    SECTION("crossing bars 1.0 apart in height are a violation") {
        auto [x, y] = crossing(1.0);
        const auto v = judge(x, {y});
        REQUIRE(v.size() == 1);
        CHECK(v[0].dist == Approx(1.0));
        CHECK(v[0].required == Approx(LIMIT));
        CHECK((v[0].p_new - Vec3d(0, 0, 0)).norm() < 1e-12);
        CHECK((v[0].p_old - Vec3d(0, 0, 1)).norm() < 1e-12);
        CHECK(v[0].with == PrimKind::head_bar);
    }
    SECTION("a slanted bar is not judged by the height difference") {
        // In the YZ plane at 45 deg, 1.5 above the X bar where they cross in plan.
        const Prim x = bar({-5., 0., 10.}, {5., 0., 10.});
        const Prim y = bar({0., -3., 8.5}, {0., 3., 14.5});
        CHECK(capsule_dist(x.cap, y.cap) == Approx(1.5 * std::sqrt(0.5)));
        const auto v = judge(x, {y});
        REQUIRE(v.size() == 1);
        CHECK(v[0].dist == Approx(1.0607).epsilon(1e-4));
    }
    SECTION("exactly at the limit is clear") {
        auto [x, y] = crossing(1.2);
        CHECK(capsule_dist(x.cap, y.cap) == LIMIT);
        CHECK(judge(x, {y}).empty());
        CHECK(judge(crossing(LIMIT + DELTA).first, {crossing(LIMIT + DELTA).second}).empty());
        CHECK(judge(crossing(LIMIT - DELTA).first, {crossing(LIMIT - DELTA).second}).size() == 1);
    }
    SECTION("one ulp under the limit is still found by the index") {
        const double d = std::nextafter(LIMIT, 0.);
        const Prim x = bar({0., 0., 0.}, {10., 0., 0.});
        CHECK(judge(x, {bar({0., d, 0.}, {10., d, 0.})}).size() == 1);
        CHECK(judge(x, {bar({0., LIMIT, 0.}, {10., LIMIT, 0.})}).empty());
        // Found by search: here the box edges round apart although the gap,
        // 0.6, is under the limit, 0.6000000000000001. Without EPS_BROAD the
        // index would miss it.
        const double yc = -0.525424321409389, yo = 0.07457567859061102;
        REQUIRE(std::abs(yo - yc) < 0.3 + 0.1 + SAFETY_CLEARANCE);
        REQUIRE(yo - 0.1 > yc + (0.3 + SAFETY_CLEARANCE));
        CHECK(judge(bar({0., yc, 0.}, {10., yc, 0.}, 0.3), {bar({0., yo, 0.}, {10., yo, 0.}, 0.1)}).size() == 1);
    }
    SECTION("L1 judges without the clearance") {
        auto [x, y] = crossing(1.1); // radii sum 1.0
        CHECK(judge(x, {y}, 0.).empty());
        CHECK(judge(x, {y}, SAFETY_CLEARANCE).size() == 1);
    }
}

TEST_CASE("degenerate segments", "[bridge_clearance]")
{
    SECTION("a zero-length pillar is a point") {
        const Capsule p = pillar_capsule({0, 0, 1}, 0., 0.5, 0.5);
        CHECK(capsule_dist(p, bar_capsule({-5, 0, 2}, {5, 0, 2}, 0.5)) == Approx(1.));
        CHECK(capsule_dist(p, p) == 0.);
    }
    SECTION("two points") {
        CHECK(capsule_dist(bar_capsule({0, 0, 0}, {0, 0, 0}, 0.5),
                           bar_capsule({3, 4, 0}, {3, 4, 0}, 0.5)) == Approx(5.));
    }
    SECTION("parallel bars") {
        const Capsule a = bar_capsule({0, 0, 0}, {10, 0, 0}, 0.5);
        CHECK(capsule_dist(a, bar_capsule({2, 1, 0}, {8, 1, 0}, 0.5)) == Approx(1.));
        CHECK(capsule_dist(a, bar_capsule({12, 1, 0}, {20, 1, 0}, 0.5)) == Approx(std::sqrt(5.)));
        CHECK(capsule_dist(a, bar_capsule({5, 0, 0}, {15, 0, 0}, 0.5)) == 0.); // collinear overlap
    }
    SECTION("the closest points agree with the distance") {
        std::mt19937 gen(0xB00);
        std::uniform_real_distribution<double> u(-5., 5.);
        for (int i = 0; i < 1000; ++i) {
            const Vec3d p1{u(gen), u(gen), u(gen)}, q1{u(gen), u(gen), u(gen)};
            const Vec3d p2{u(gen), u(gen), u(gen)}, q2 = i % 7 ? Vec3d{u(gen), u(gen), u(gen)} : p2;
            double s, t;
            const double d = bracing::geom::seg_seg_closest3d(p1, q1, p2, q2, s, t);
            CHECK(d == bracing::geom::seg_seg_dist3d(p1, q1, p2, q2));
            CHECK(s >= 0.); CHECK(s <= 1.); CHECK(t >= 0.); CHECK(t <= 1.);
            CHECK(((p1 + s * (q1 - p1)) - (p2 + t * (q2 - p2))).norm() == Approx(d).margin(1e-12));
        }
    }
}

TEST_CASE("bars and pillars in each other's way", "[bridge_clearance]")
{
    SECTION("two head bars running through each other") {
        const Prim old = bar({-5, 0, 10}, {5, 0, 10}, 0.5, 1, -1, 7);
        const Prim cand = bar({0, -5, 10.358}, {0, 5, 10.358}, 0.5, 2, -1, 8);
        const auto v = judge(cand, {old});
        REQUIRE(v.size() == 1);
        CHECK(v[0].dist == Approx(0.358));
        CHECK(v[0].with_head == 1);
        CHECK(v[0].with_pillar == 7); // where the head bar ends, not -1
    }
    SECTION("the pillar a contact names") {
        const Prim cand = bar({0, -5, 10}, {0, 5, 10}, 0.5, 2);
        // A route bar between two pillars: the one at its end.
        CHECK(judge(cand, {bar({-5, 0, 10}, {5, 0, 10}, 0.5, 1, 3, 4, PrimKind::route_bar)})[0].with_pillar == 4);
        // A prop bar known by its start only.
        CHECK(judge(cand, {bar({-5, 0, 10}, {5, 0, 10}, 0.5, -1, 3, -1, PrimKind::prop_bar)})[0].with_pillar == 3);
        // A pillar names itself, a pinhead its carrier.
        CHECK(judge(cand, {pillar(6, 0, 0, 0, 20)})[0].with_pillar == 6);
        CHECK(judge(cand, {pinhead({0, 0, 11}, {0, 0, 9}, 0.5, 3, 9)})[0].with_pillar == 9);
    }
    SECTION("a bar through a third pillar") {
        const Prim B = pillar(5, 0, 0.79, 0, 20);
        const auto v = judge(bar({-5, 0, 10}, {5, 0, 10}, 0.5, 2, -1, 8), {B});
        REQUIRE(v.size() == 1);
        CHECK(v[0].dist == Approx(0.79));
        CHECK(v[0].with == PrimKind::pillar);
        CHECK(v[0].with_pillar == 5);
    }
    SECTION("a bar under a pillar standing on the model, 5 mm below its foot") {
        const Prim B = pillar(5, 0, 0, 20, 10);
        CHECK(judge(bar({-5, 0, 15}, {5, 0, 15}, 0.5, 2, -1, 8), {B}).empty());
    }
    SECTION("a bar through someone else's pinhead") {
        const Prim k = pinhead({0, 0.5, 11}, {0, 0.5, 9}, 0.5, 3, 9);
        const auto v = judge(bar({-5, 0, 10}, {5, 0, 10}, 0.5, 2, -1, 8), {k});
        REQUIRE(v.size() == 1);
        CHECK(v[0].dist == Approx(0.5));
        CHECK(v[0].with == PrimKind::pinhead);
    }
    SECTION("a new pillar piercing a committed bar") {
        const Prim old = bar({-5, 0, 10}, {5, 0, 10}, 0.5, 1, -1, 7);
        const auto v = judge(pillar(6, 0, 0.6, 0, 20, 0.5, 2), {old});
        REQUIRE(v.size() == 1);
        CHECK(v[0].dist == Approx(0.6));
    }
    SECTION("pillar against pillar and pinhead against pinhead are not judged") {
        CHECK(judge(pillar(1, 0, 0, 0, 10), {pillar(2, 0.1, 0, 0, 10)}).empty());
        CHECK(judge(pinhead({0, 0, 10}, {0, 0, 8}, 0.5, 1, -1),
                    {pinhead({0.1, 0, 10}, {0.1, 0, 8}, 0.5, 2, -1)}).empty());
        CHECK_FALSE(judged_kinds(PrimKind::pillar, PrimKind::pillar));
        CHECK_FALSE(judged_kinds(PrimKind::pinhead, PrimKind::pinhead));
        CHECK(judged_kinds(PrimKind::pillar, PrimKind::pinhead));
    }
}

TEST_CASE("exemptions are by id", "[bridge_clearance]")
{
    SECTION("one head's path") {
        const Prim own_bar = bar({0, 0, 10}, {3, 0, 10}, 0.5, 4, -1, 7);
        const Prim own_head = pinhead({0, 0, 12}, {0, 0, 10}, 0.5, 4, 7);
        CHECK(exempt(own_bar, own_head));
        CHECK(judge(own_bar, {own_head}).empty());
        Prim other = own_head;
        other.head_id = 5;
        other.pillar_a = 9;
        CHECK_FALSE(exempt(own_bar, other));
        CHECK(judge(own_bar, {other}).size() == 1);
    }
    SECTION("a bar and the pillars it ends on") {
        const Prim P = pillar(7, 3, 0, 0, 10);
        CHECK(judge(bar({0, 0, 10}, {3, 0, 10}, 0.5, 4, -1, 7), {P}).empty());
        CHECK(judge(bar({3, 0, 5}, {6, 0, 7}, 0.5, -1, 7, 8, PrimKind::route_bar), {P}).empty());
        CHECK(judge(P, {bar({0, 0, 10}, {3, 0, 10}, 0.5, 4, -1, 7)}).empty());
        // Ending there in geometry is not enough.
        CHECK(judge(bar({0, 0, 10}, {3, 0, 10}, 0.5, 4, -1, 8), {P}).size() == 1);
    }
    SECTION("a pillar and its own head's bar") {
        const Prim own = pillar(6, 0, 0, 0, 10, 0.5, 4);
        CHECK(judge(bar({-3, 0, 5}, {3, 0, 5}, 0.5, 4, -1, 9), {own}).empty());
        CHECK(judge(bar({-3, 0, 5}, {3, 0, 5}, 0.5, 5, -1, 9), {own}).size() == 1);
    }
    SECTION("a pinhead and its carrier") {
        const Prim P = pillar(7, 0, 0, 0, 10);
        CHECK(judge(pinhead({0, 0, 12}, {0, 0, 10}, 0.5, 4, 7), {P}).empty());
        CHECK(judge(pinhead({0, 0, 12}, {0, 0, 10}, 0.5, 4, 8), {P}).size() == 1);
    }
    SECTION("shared pillars, ascending, never -1") {
        CHECK(shared_pillars(bar(ORIGIN, ORIGIN, .5, -1, 3, 7), bar(ORIGIN, ORIGIN, .5, -1, 7, 3)) == std::vector<long>{3, 7});
        CHECK(shared_pillars(bar(ORIGIN, ORIGIN, .5, -1, -1, 7), bar(ORIGIN, ORIGIN, .5, -1, -1, -1)).empty());
        CHECK(shared_pillars(bar(ORIGIN, ORIGIN, .5, -1, -1, 7), pinhead(ORIGIN, ORIGIN, .5, 2, 7)) == std::vector<long>{7});
    }
}

TEST_CASE("joint radius R", "[bridge_clearance]")
{
    CHECK(joint_radius(0.5, 0.5, 0.5) == Approx(1.7));
    CHECK(joint_radius(0.75, 0.75, 0.75) == Approx(2.45));
    CHECK(joint_radius(1.0, 0.25, 0.4) == Approx(1.85)); // per pair, not fixed

    SECTION("the cylinder grows with the radii") {
        // A 0.75 bar from 5 mm away to the axis of a 0.75 pillar, judged against
        // another 0.75 bar: R = 2.45, so 5 (1 - t) >= 2.45 is what stays.
        const Capsule x = bar_capsule({5, 0, 10}, {0, 0, 10}, 0.75);
        const auto pcs = clip_joints(x, 0.75, {{Vec2d(0, 0), 0.75}});
        REQUIRE(pcs.size() == 1);
        CHECK(pcs[0].t0 == 0.);
        CHECK(pcs[0].t1 == Approx(1. - 2.45 / 5.));
    }
}

TEST_CASE("bars meeting on a shared pillar", "[bridge_clearance]")
{
    Axes axes;
    axes.m[7] = {Vec2d(0, 0), 0.5};
    axes.m[8] = {Vec2d(0, 0), 0.5};
    const Vec3d joint{0, 0, 10};

    SECTION("90 degrees apart at one point: clear once the joint is clipped") {
        const Prim a = bar(polar3(0, 5, 10), joint, 0.5, 1, -1, 7);
        const Prim b = bar(polar3(90, 5, 10), joint, 0.5, 2, -1, 7);
        CHECK(capsule_dist(a.cap, b.cap) == 0.);
        CHECK(joint_clipped_dist(a.cap, b.cap, {axes.m[7]}) == Approx(1.7 * std::sqrt(2.)));
        CHECK(judge(b, {a}, SAFETY_CLEARANCE, axes.fn()).empty());
    }
    SECTION("10 degrees apart, 5 mm long: a violation beyond the joint") {
        const Prim a = bar(polar3(0, 5, 10), joint, 0.5, 1, -1, 7);
        const Prim b = bar(polar3(10, 5, 10), joint, 0.5, 2, -1, 7);
        const auto v = judge(b, {a}, SAFETY_CLEARANCE, axes.fn());
        REQUIRE(v.size() == 1);
        CHECK(v[0].dist == Approx(2. * 1.7 * std::sin(5. * PI / 180.))); // ~0.30
        CHECK(v[0].dist == Approx(0.30).margin(0.005));
        // The contact is outside the cylinder, on what is left.
        CHECK(v[0].p_new.head<2>().norm() >= 1.7 - 1e-9);
        CHECK(v[0].p_old.head<2>().norm() >= 1.7 - 1e-9);
    }
    SECTION("ending near P is not sharing P: the ids decide") {
        const Prim a = bar(polar3(0, 5, 10), joint, 0.5, 1, -1, 7);
        const Prim b = bar(polar3(90, 5, 10), joint, 0.5, 2, -1, 8); // owner_b is Q
        CHECK(shared_pillars(a, b).empty());
        const auto v = judge(b, {a}, SAFETY_CLEARANCE, axes.fn());
        REQUIRE(v.size() == 1);
        CHECK(v[0].dist == Approx(0.).margin(1e-12));
    }
    SECTION("a bar clipped away entirely passes") {
        const Prim a = bar(polar3(0, 1.7, 10), joint, 0.5, 1, -1, 7); // ends on the surface
        const Prim b = bar(polar3(5, 5, 10), joint, 0.5, 2, -1, 7);
        CHECK(clip_joints(a.cap, 0.5, {axes.m[7]}).empty());
        CHECK(joint_clipped_dist(a.cap, b.cap, {axes.m[7]}) == INF);
        CHECK(judge(b, {a}, SAFETY_CLEARANCE, axes.fn()).empty());
    }
    SECTION("a pinhead sharing the carrier is clipped too") {
        axes.m[9] = {Vec2d(0, 0), 0.5};
        // Head 3 sits on pillar 9; another head's bar ends on 9's top.
        const Prim k = pinhead({0, 0, 12}, joint, 0.5, 3, 9);
        const Prim b = bar(polar3(0, 5, 10), joint, 0.5, 2, -1, 9);
        CHECK(judge(b, {k}, SAFETY_CLEARANCE, axes.fn()).empty());
    }
    SECTION("two bars sharing both end pillars are clipped at both ends") {
        axes.m[8] = {Vec2d(10, 0), 0.5};
        const Prim a = bar({0, 0, 10}, {10, 0, 10}, 0.5, 1, 7, 8, PrimKind::route_bar);
        const Prim up = bar({0, 0, 10}, {10, 0, 20}, 0.5, 2, 7, 8, PrimKind::route_bar);
        CHECK(shared_pillars(a, up) == std::vector<long>{7, 8});
        // 45 deg apart, both ends joint: what is left is 1.7 apart.
        CHECK(joint_clipped_dist(a.cap, up.cap, {axes.m[7], axes.m[8]}) == Approx(1.7));
        CHECK(judge(up, {a}, SAFETY_CLEARANCE, axes.fn()).empty());
        // Crossing halfway, far from both joints, is a violation.
        const Prim cross = bar({0, 0, 12}, {10, 0, 8}, 0.5, 2, 7, 8, PrimKind::route_bar);
        const auto v = judge(cross, {a}, SAFETY_CLEARANCE, axes.fn());
        REQUIRE(v.size() == 1);
        CHECK(v[0].dist == Approx(0.).margin(1e-12));
        CHECK(v[0].p_new.isApprox(Vec3d(5, 0, 10)));
    }
    SECTION("no shared pillar: the plain distance") {
        const Capsule x = bar_capsule({-5, 0, 0}, {5, 0, 0}, 0.5);
        const Capsule y = bar_capsule({0, -5, 1}, {0, 5, 1}, 0.5);
        CHECK(joint_clipped_dist(x, y, {}) == capsule_dist(x, y));
    }
}

TEST_CASE("clipping by the vertical cylinder", "[bridge_clearance]")
{
    double t0, t1;
    const Vec2d O(0, 0);

    SECTION("crossing it") {
        REQUIRE(cylinder_interval({-5, 0, 0}, {5, 0, 3}, O, 1., t0, t1));
        CHECK(t0 == Approx(0.4));
        CHECK(t1 == Approx(0.6));
    }
    SECTION("all inside") {
        REQUIRE(cylinder_interval({-0.5, 0, 0}, {0.5, 0, 9}, O, 1., t0, t1));
        CHECK(t0 == 0.);
        CHECK(t1 == 1.);
        CHECK(clip_joints(bar_capsule({-0.3, 0, 0}, {0.3, 0, 9}, 0.1), 0.1, {{O, 0.1}}).empty());
    }
    SECTION("missing it") {
        CHECK_FALSE(cylinder_interval({-5, 2, 0}, {5, 2, 0}, O, 1., t0, t1));
        CHECK_FALSE(cylinder_interval({2, 0, 0}, {5, 0, 0}, O, 1., t0, t1));
    }
    SECTION("tangent: nothing is clipped") {
        CHECK_FALSE(cylinder_interval({-5, 1, 0}, {5, 1, 0}, O, 1., t0, t1));
        // R = 0.3 + 0 + 0 + 0.2 = 0.5 exactly.
        const auto pcs = clip_joints(bar_capsule({-5, 0.5, 0}, {5, 0.5, 0}, 0.), 0., {{O, 0.3}});
        REQUIRE(pcs.size() == 1);
        CHECK(pcs[0].t0 == 0.);
        CHECK(pcs[0].t1 == 1.);
    }
    SECTION("an end exactly on the surface, the rest outside: nothing is clipped") {
        CHECK_FALSE(cylinder_interval({1, 0, 0}, {5, 0, 0}, O, 1., t0, t1));
    }
    SECTION("nearly vertical segments, just above the point threshold") {
        // Plan length 2e-6 mm, so A = 4e-12 > EPS_LEN: the quadratic is solved.
        REQUIRE(cylinder_interval({0.5, 0, 0}, {0.5 + 2e-6, 0, 9}, O, 1., t0, t1));
        CHECK(t0 == 0.);
        CHECK(t1 == 1.);
        CHECK_FALSE(cylinder_interval({1., 0, 0}, {1. + 2e-6, 0, 9}, O, 1., t0, t1)); // leaving from the surface
        REQUIRE(cylinder_interval({1. - 1e-6, 0, 0}, {1. + 1e-6, 0, 9}, O, 1., t0, t1)); // crossing it
        CHECK(t0 == 0.);
        CHECK(t1 == Approx(0.5).margin(1e-6));
    }
    SECTION("no jump at the vertical threshold") {
        // Straddling the surface, plan length 8e-7 (A = 6.4e-13 <= EPS_LEN):
        // split where the plan distance crosses R, as just above the threshold.
        REQUIRE(cylinder_interval({1. - 4e-7, 0, 0}, {1. + 4e-7, 0, 9}, O, 1., t0, t1));
        CHECK(t0 == 0.);
        CHECK(t1 == Approx(0.5).margin(1e-6));
        REQUIRE(cylinder_interval({1. + 4e-7, 0, 0}, {1. - 4e-7, 0, 9}, O, 1., t0, t1));
        CHECK(t0 == Approx(0.5).margin(1e-6));
        CHECK(t1 == 1.);
    }
    SECTION("vertical segments are judged by their ends") {
        REQUIRE(cylinder_interval({0.5, 0, 0}, {0.5, 0, 9}, O, 1., t0, t1));
        CHECK(t0 == 0.);
        CHECK(t1 == 1.);
        CHECK(cylinder_interval({1, 0, 0}, {1, 0, 9}, O, 1., t0, t1)); // on the surface counts
        CHECK_FALSE(cylinder_interval({1.5, 0, 0}, {1.5, 0, 9}, O, 1., t0, t1));
    }
    SECTION("two shared pillars leave at most three pieces") {
        // R = 0.1 + 0.1 + 0.1 + 0.2 = 0.5 around x = 2.5 and x = 7.5 of a 10 mm bar.
        const Capsule x = bar_capsule({0, 0, 0}, {10, 0, 0}, 0.1);
        const auto pcs = clip_joints(x, 0.1, {{Vec2d(2.5, 0), 0.1}, {Vec2d(7.5, 0), 0.1}});
        REQUIRE(pcs.size() == 3);
        CHECK(pcs[0].t0 == 0.);
        CHECK(pcs[0].t1 == Approx(0.2));
        CHECK(pcs[1].t0 == Approx(0.3));
        CHECK(pcs[1].t1 == Approx(0.7));
        CHECK(pcs[2].t0 == Approx(0.8));
        CHECK(pcs[2].t1 == 1.);
    }
    SECTION("overlapping cylinders merge") {
        // R = 0.5 around x = 5 and x = 5.5: [0.45, 0.55] and [0.5, 0.6].
        const Capsule x = bar_capsule({0, 0, 0}, {10, 0, 0}, 0.1);
        const auto pcs = clip_joints(x, 0.1, {{Vec2d(5, 0), 0.1}, {Vec2d(5.5, 0), 0.1}});
        REQUIRE(pcs.size() == 2);
        CHECK(pcs[0].t1 == Approx(0.45));
        CHECK(pcs[1].t0 == Approx(0.6));
    }
}

TEST_CASE("the index finds what comparing every pair finds", "[bridge_clearance]")
{
    std::mt19937 gen(0x5EED); // fixed seed - a failure has to be reproducible
    std::uniform_real_distribution<double> coord(0., 30.), len(0., 8.), rad(0.1, 0.8), unit(0., 1.);
    std::uniform_int_distribution<long> head_id(-1, 40), pillar_id(-1, 25);
    std::uniform_int_distribution<int> kind(0, 4);

    Axes axes;
    for (long id = 0; id <= 25; ++id) axes.m[id] = {Vec2d(coord(gen), coord(gen)), rad(gen)};

    auto random_prim = [&]() {
        Prim p;
        p.kind = PrimKind(kind(gen));
        p.head_id = head_id(gen);
        const Vec3d a{coord(gen), coord(gen), coord(gen)};
        switch (p.kind) {
        case PrimKind::pillar:
            p.pillar_a = std::uniform_int_distribution<long>(0, 25)(gen);
            // Some of zero height.
            p.cap = pillar_capsule(a, unit(gen) < 0.1 ? 0. : len(gen) * 3., rad(gen), rad(gen));
            break;
        case PrimKind::pinhead:
            p.pillar_a = pillar_id(gen);
            p.cap = pinhead_capsule(a, a - Vec3d(0.3 * unit(gen), 0.3 * unit(gen), 1.6), rad(gen));
            break;
        default: {
            p.pillar_a = pillar_id(gen);
            p.pillar_b = pillar_id(gen);
            // Start some bars on a pillar's axis, as bars do, and make some vertical.
            Vec3d s = a;
            if (p.pillar_a >= 0 && unit(gen) < 0.5) s.head<2>() = axes.m[p.pillar_a].xy;
            Vec3d e = s + Vec3d(len(gen) - 4., len(gen) - 4., len(gen) - 4.);
            if (unit(gen) < 0.1) e.head<2>() = s.head<2>();
            if (p.pillar_b >= 0 && unit(gen) < 0.5) e.head<2>() = axes.m[p.pillar_b].xy;
            p.cap = kind(gen) == 0 ? diffbridge_capsule(s, e, rad(gen), rad(gen)) :
                                     bar_capsule(s, e, rad(gen));
        }
        }
        return p;
    };

    ClearanceIndex idx;
    for (int i = 0; i < 600; ++i) idx.insert(random_prim());
    REQUIRE(idx.size() == 600);

    std::vector<Prim> cands;
    for (int i = 0; i < 300; ++i) cands.push_back(random_prim());

    for (double c : {SAFETY_CLEARANCE, 0.}) {
        const std::vector<Contact> got = idx.violations(cands, c, axes.fn());

        std::vector<Contact> brute;
        for (size_t i = 0; i < cands.size(); ++i)
            for (size_t j = 0; j < idx.prims().size(); ++j) {
                Contact ct;
                if (pair_contact(cands[i], idx.prims()[j], c, axes.fn(), ct)) {
                    ct.cand = i;
                    ct.with_index = j;
                    brute.push_back(ct);
                }
            }

        INFO("clearance " << c);
        REQUIRE(brute.size() > 50); // the test means something
        REQUIRE(got.size() == brute.size());
        for (size_t k = 0; k < got.size(); ++k) {
            CHECK(got[k].cand == brute[k].cand);
            CHECK(got[k].with_index == brute[k].with_index);
            CHECK(got[k].dist == brute[k].dist);
        }

        // An oracle independent of the clipping: with no shared pillar the
        // verdict is the plain distance, exempt pairs never report, and a
        // shared pillar can only clear a pair.
        size_t contact_k = 0;
        for (size_t i = 0; i < cands.size(); ++i)
            for (size_t j = 0; j < idx.prims().size(); ++j) {
                const Prim &x = cands[i], &y = idx.prims()[j];
                const bool reported = contact_k < brute.size() && brute[contact_k].cand == i &&
                                      brute[contact_k].with_index == j;
                if (reported) ++contact_k;
                const bool close = capsule_dist(x.cap, y.cap) < x.cap.r + y.cap.r + c;
                if (!judged_kinds(x.kind, y.kind) || exempt(x, y))
                    CHECK_FALSE(reported);
                else if (shared_pillars(x, y).empty())
                    CHECK(reported == close);
                else if (reported)
                    CHECK(close); // consistency only; the clipping is checked by sampling

            }
        CHECK(contact_k == brute.size());

        // And the exemptions and joints do take some pairs out.
        size_t raw = 0;
        for (const Prim &x : cands)
            for (const Prim &y : idx.prims())
                if (judged_kinds(x.kind, y.kind) && capsule_dist(x.cap, y.cap) < x.cap.r + y.cap.r + c)
                    ++raw;
        CHECK(raw > brute.size());
    }
}

TEST_CASE("clipping agrees with sampling", "[bridge_clearance]")
{
    // An oracle that shares no code with clip_joints: every sample point is
    // kept exactly when it is outside every joint cylinder. Samples within
    // 1e-7 mm of a cylinder's surface are not judged.
    std::mt19937 gen(0xC11F);
    std::uniform_real_distribution<double> u(-4., 4.), rad(0.1, 0.8), unit(0., 1.);

    size_t judged = 0, kept = 0;
    for (int it = 0; it < 2000; ++it) {
        std::vector<JointAxis> shared{{Vec2d(0, 0), rad(gen)}};
        if (unit(gen) < 0.4) shared.push_back({Vec2d(u(gen), u(gen)), rad(gen)});

        Vec3d a{u(gen), u(gen), u(gen)}, b{u(gen), u(gen), u(gen)};
        if (unit(gen) < 0.1) b.head<2>() = a.head<2>() + Vec2d(1e-7 * unit(gen), 0.); // vertical
        const Capsule x = bar_capsule(a, b, rad(gen));
        const double r_other = rad(gen);
        const std::vector<Piece> pcs = clip_joints(x, r_other, shared);

        for (int k = 0; k <= 1000; ++k) {
            const double t = k / 1000.;
            const Vec2d p = (a + t * (b - a)).head<2>();
            bool inside = false, near = false;
            for (const JointAxis &P : shared) {
                const double R = P.r + x.r + r_other + 0.2, dist = (p - P.xy).norm();
                if (std::abs(dist - R) < 1e-7) near = true;
                if (dist < R) inside = true;
            }
            if (near) continue;
            bool in_piece = false;
            for (const Piece &pc : pcs) in_piece = in_piece || (t >= pc.t0 && t <= pc.t1);
            ++judged;
            kept += in_piece;
            CHECK(in_piece == !inside);
        }
    }
    CHECK(kept > 0);
    CHECK(kept < judged);
}

TEST_CASE("violations come in one fixed order", "[bridge_clearance]")
{
    ClearanceIndex idx;
    for (int i = 0; i < 5; ++i) idx.insert(bar({-5, double(i) * 0.1, 10}, {5, double(i) * 0.1, 10}, 0.5, i));
    const std::vector<Prim> cands{bar({0, -5, 10}, {0, 5, 10}, 0.5, 10), bar({1, -5, 10}, {1, 5, 10}, 0.5, 11)};
    const auto v = idx.violations(cands, SAFETY_CLEARANCE, no_axes);
    REQUIRE(v.size() == 10);
    for (size_t k = 0; k < v.size(); ++k) {
        CHECK(v[k].cand == k / 5);
        CHECK(v[k].with_index == k % 5);
    }
}

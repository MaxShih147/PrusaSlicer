// BracingPlanner (openspec change sla-bracing-planar-topology, task group 3).
//
// Thresholds are probed at limit - 1e-6 and limit + 1e-6, and where the
// arithmetic is exact, at the limit itself: the spec rejects only on
// "strictly less than".

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/SLA/BracingPlanner.hpp"

using namespace Slic3r;
using namespace Slic3r::sla;
using namespace Slic3r::sla::bracing;
using Catch::Approx;

namespace {

constexpr double LIMIT = 0.5 + 0.5 + SAFETY_CLEARANCE_MM; // two 0.5 mm bars
constexpr double DELTA = 1e-6;

Vec2d polar(double deg, double len)
{
    return {len * std::cos(deg * PI / 180.), len * std::sin(deg * PI / 180.)};
}

PillarGeom mk(long id, double x, double y, double height = 20., double r = 0.5,
              double foot = 0.)
{
    PillarGeom p;
    p.id = id; p.xy = {x, y}; p.foot_z = foot; p.height = height; p.r = r;
    return p;
}

PillarGeom frozen(PillarGeom p, unsigned pairs, unsigned links = 1)
{
    p.frozen = true; p.pairs = pairs; p.links = links;
    return p;
}

PlannerConfig config()
{
    PlannerConfig cfg;
    cfg.max_link_distance   = 10.;
    cfg.min_pillar_distance = 1.;
    cfg.truss.bridge_slope_rad = PI / 4;
    cfg.truss.ground_z    = 0.;
    cfg.truss.base_height = 1.;
    cfg.truss.base_radius = 2.;
    cfg.truss.mode = PillarConnectionMode::dynamic;
    return cfg;
}

std::string str(const Decision &d)
{
    return std::to_string(d.edge.a) + "-" + std::to_string(d.edge.b) + ":" +
           reason_name(d.reason);
}

std::vector<std::string> strs(const std::vector<Decision> &ds)
{
    std::vector<std::string> out;
    for (const Decision &d : ds) out.push_back(str(d));
    return out;
}

const Decision *find(const std::vector<Decision> &ds, long a, long b)
{
    for (const Decision &d : ds)
        if (d.edge.a == std::min(a, b) && d.edge.b == std::max(a, b)) return &d;
    return nullptr;
}

Segment3 seg(Vec3d a, Vec3d b, double r = 0.5) { return {a, b, r}; }

} // namespace

// ─── 3.1 constants ───────────────────────────────────────────────────────────

TEST_CASE("bracing constants", "[bracing_planner]")
{
    REQUIRE(SAFETY_CLEARANCE_MM == 0.2);
    REQUIRE(PAIR_CAP == 3u);
    REQUIRE(RESCUE_HEIGHT_MM == 15.);
    // The literal is the correctly rounded cos(20 deg).
    REQUIRE(COS_MIN_SHARED_ANGLE == std::cos(20. * PI / 180.));
    PlannerConfig cfg;
    REQUIRE(cfg.cos_min_shared_ang == COS_MIN_SHARED_ANGLE);
    REQUIRE(cfg.pair_cap == PAIR_CAP);
}

// ─── 3.2 plan-view distances ─────────────────────────────────────────────────

TEST_CASE("point to segment in plan view", "[bracing_planner]")
{
    using geom::point_seg_dist2d;
    const Vec2d a{0, 0}, b{10, 0};
    REQUIRE(point_seg_dist2d({5, 3}, a, b) == Approx(3.));
    REQUIRE(point_seg_dist2d({-3, 4}, a, b) == Approx(5.));  // before a
    REQUIRE(point_seg_dist2d({13, -4}, a, b) == Approx(5.)); // past b
    REQUIRE(point_seg_dist2d({4, 0}, a, b) == 0.);
    // A zero-length segment is a point.
    REQUIRE(point_seg_dist2d({3, 4}, a, a) == Approx(5.));
    REQUIRE(point_seg_dist2d({3, 4}, a, Vec2d{1e-7, 0}) == Approx(5.).margin(1e-6));
}

TEST_CASE("segment to segment in plan view", "[bracing_planner]")
{
    using geom::seg_seg_dist2d;

    SECTION("crossing, T and touching ends are 0") {
        REQUIRE(seg_seg_dist2d({0, 0}, {6, 6}, {0, 6}, {6, 0}) == 0.);
        REQUIRE(seg_seg_dist2d({0, 0}, {8, 0}, {4, 5}, {4, 0}) == 0.);
        REQUIRE(seg_seg_dist2d({0, 0}, {4, 0}, {4, 0}, {4, 4}) == 0.);
    }
    SECTION("T that stops short") {
        REQUIRE(seg_seg_dist2d({0, 0}, {8, 0}, {4, 5}, {4, 0.1}) == Approx(0.1));
    }
    SECTION("parallel") {
        REQUIRE(seg_seg_dist2d({0, 0}, {10, 0}, {3, 2}, {7, 2}) == Approx(2.));
        REQUIRE(seg_seg_dist2d({0, 0}, {10, 0}, {12, 2}, {15, 2}) ==
                Approx(std::hypot(2., 2.)));
    }
    SECTION("collinear") {
        REQUIRE(seg_seg_dist2d({0, 0}, {6, 0}, {3, 0}, {9, 0}) == 0.);  // overlapping
        REQUIRE(seg_seg_dist2d({0, 0}, {10, 0}, {3, 0}, {7, 0}) == 0.); // contained
        REQUIRE(seg_seg_dist2d({0, 0}, {4, 0}, {6, 0}, {10, 0}) == Approx(2.)); // apart
    }
    SECTION("zero length") {
        REQUIRE(seg_seg_dist2d({0, 0}, {0, 0}, {3, 4}, {3, 4}) == Approx(5.));
        REQUIRE(seg_seg_dist2d({5, 2}, {5, 2}, {0, 0}, {10, 0}) == Approx(2.));
        REQUIRE(seg_seg_dist2d({5, 0}, {5, 0}, {0, 0}, {10, 0}) == 0.);
    }
    SECTION("almost parallel, not crossing") {
        REQUIRE(seg_seg_dist2d({0, 0}, {10, 0}, {0, 1}, {10, 1 - 1e-9}) ==
                Approx(1. - 1e-9));
    }
    SECTION("symmetric") {
        const Vec2d a{0.3, -1}, b{7, 2.5}, c{1, 4}, d{6, 1.7};
        REQUIRE(seg_seg_dist2d(a, b, c, d) == Approx(seg_seg_dist2d(c, d, a, b)));
        REQUIRE(seg_seg_dist2d(a, b, c, d) == Approx(seg_seg_dist2d(b, a, d, c)));
    }
}

TEST_CASE("plan-view thresholds at limit +- 1e-6", "[bracing_planner]")
{
    using geom::seg_seg_dist2d;
    using geom::point_seg_dist2d;
    for (double g : {LIMIT - DELTA, LIMIT, LIMIT + DELTA}) {
        const double d = seg_seg_dist2d({0, 0}, {10, 0}, {3, g}, {7, g});
        const double p = point_seg_dist2d({5, g}, {0, 0}, {10, 0});
        REQUIRE((d < LIMIT) == (g < LIMIT));
        REQUIRE((p < LIMIT) == (g < LIMIT));
    }
}

TEST_CASE("closest points in plan view", "[bracing_planner]")
{
    double s, t;
    REQUIRE(geom::seg_seg_closest2d({0, 0}, {6, 6}, {0, 6}, {6, 0}, s, t) == 0.);
    REQUIRE(s == Approx(0.5));
    REQUIRE(t == Approx(0.5));
    REQUIRE(geom::seg_seg_closest2d({0, 0}, {8, 0}, {4, 5}, {4, 1}, s, t) == Approx(1.));
    REQUIRE(s == Approx(0.5));
    REQUIRE(t == Approx(1.));
}

// ─── 3.3 3D distance, height difference, angle ───────────────────────────────

TEST_CASE("segment to segment in 3D (Ericson)", "[bracing_planner]")
{
    using geom::seg_seg_dist3d;

    SECTION("perpendicular skew lines") {
        REQUIRE(seg_seg_dist3d({-5, 0, 0}, {5, 0, 0}, {0, -5, 2}, {0, 5, 2}) == Approx(2.));
    }
    SECTION("parallel") {
        REQUIRE(seg_seg_dist3d({0, 0, 0}, {10, 0, 0}, {2, 0, 3}, {8, 0, 3}) == Approx(3.));
        REQUIRE(seg_seg_dist3d({0, 0, 0}, {10, 0, 0}, {12, 0, 0}, {15, 0, 0}) == Approx(2.));
    }
    SECTION("closest at the ends") {
        REQUIRE(seg_seg_dist3d({0, 0, 0}, {1, 0, 0}, {3, 0, 4}, {3, 5, 4}) ==
                Approx(std::hypot(2., 4.)));
    }
    SECTION("zero length") {
        REQUIRE(seg_seg_dist3d({0, 0, 0}, {0, 0, 0}, {3, 4, 0}, {3, 4, 0}) == Approx(5.));
        REQUIRE(seg_seg_dist3d({5, 2, 0}, {5, 2, 0}, {0, 0, 0}, {10, 0, 0}) == Approx(2.));
        REQUIRE(seg_seg_dist3d({0, 0, 0}, {10, 0, 0}, {5, 0, 3}, {5, 0, 3}) == Approx(3.));
    }
    SECTION("intersecting") {
        REQUIRE(seg_seg_dist3d({0, 0, 0}, {4, 4, 4}, {4, 0, 0}, {0, 4, 4}) ==
                Approx(0.).margin(1e-12));
    }
}

TEST_CASE("horizontal bar against a 45 degree bar 1.5 mm apart in height", "[bracing_planner]")
{
    // A horizontal head bar along X, and a truss bar rising at 45 degrees in
    // the YZ plane, 1.5 mm above it where they cross in plan view.
    const Segment3 head = seg({-5, 0, 0}, {5, 0, 0});
    const Segment3 bar  = seg({0, -5, 1.5 - 5}, {0, 5, 1.5 + 5});

    const double d3 = geom::seg_seg_dist3d(head.a, head.b, bar.a, bar.b);
    REQUIRE(d3 == Approx(1.5 * std::cos(PI / 4))); // about 1.06
    REQUIRE(d3 == Approx(1.0606601717798212));
    REQUIRE(d3 < LIMIT);                            // so it does not pass

    // The height difference alone would have let it through.
    const double dz = geom::plan_closest_dz(bar, head);
    REQUIRE(dz == Approx(1.5));
    REQUIRE(dz >= LIMIT);

    // |dz| * kappa, kappa = |(d1 x d2).z| / |d1 x d2|
    const Vec3d  c     = (head.b - head.a).cross(bar.b - bar.a);
    const double kappa = std::abs(c.z()) / c.norm();
    REQUIRE(d3 == Approx(dz * kappa));

    // Two horizontal bars: kappa is 1 and the gap is the height difference.
    REQUIRE(geom::seg_seg_dist3d(head.a, head.b, {0, -5, 1.5}, {0, 5, 1.5}) == Approx(1.5));
}

TEST_CASE("3D threshold at limit +- 1e-6", "[bracing_planner]")
{
    for (double g : {LIMIT - DELTA, LIMIT + DELTA}) {
        const double d = geom::seg_seg_dist3d({-5, 0, 0}, {5, 0, 0}, {0, -5, g}, {0, 5, g});
        REQUIRE((d < LIMIT) == (g < LIMIT));
    }
}

TEST_CASE("height difference where bars cross in plan view", "[bracing_planner]")
{
    const Segment3 lo = seg({0, 0, 10}, {10, 0, 0});  // z = 10 - x
    const Segment3 hi = seg({4, -3, 20}, {4, 3, 20});
    REQUIRE(geom::plan_closest_dz(hi, lo) == Approx(20. - 6.));
    REQUIRE(geom::plan_closest_dz(lo, hi) == Approx(6. - 20.));
}

TEST_CASE("shared pillar angle by cosine", "[bracing_planner]")
{
    using geom::shared_angle_ok;
    const Vec2d s{1, 2};
    auto at = [&s](double deg) { return Vec2d(s + polar(deg, 7.)); };

    REQUIRE_FALSE(shared_angle_ok(s, at(0), at(19), COS_MIN_SHARED_ANGLE));
    REQUIRE(shared_angle_ok(s, at(0), at(21), COS_MIN_SHARED_ANGLE));
    REQUIRE_FALSE(shared_angle_ok(s, at(0), at(20 - DELTA), COS_MIN_SHARED_ANGLE));
    REQUIRE(shared_angle_ok(s, at(0), at(20 + DELTA), COS_MIN_SHARED_ANGLE));
    REQUIRE(shared_angle_ok(s, at(10), at(-170), COS_MIN_SHARED_ANGLE)); // opposite
    REQUIRE_FALSE(shared_angle_ok(s, at(33), at(33), COS_MIN_SHARED_ANGLE));
    // Lengths do not matter, only directions.
    REQUIRE(shared_angle_ok(s, s + polar(0, 2.), s + polar(21, 9.), COS_MIN_SHARED_ANGLE));
    // No direction at all is never fine (and never NaN).
    REQUIRE_FALSE(shared_angle_ok(s, s, at(90), COS_MIN_SHARED_ANGLE));
}

// ─── truss layout ────────────────────────────────────────────────────────────

TEST_CASE("truss bars follow interconnect()", "[bracing_planner]")
{
    TrussParams prm = config().truss;

    SECTION("zig-zag, worked by hand") {
        // d = 4, 45 deg: zstep -4, from z 20 down to the base top at z 1.
        const auto bars = geom::truss_bars(mk(0, 0, 0), mk(1, 4, 0), prm);
        const std::vector<std::pair<Vec3d, Vec3d>> want = {
            {{0, 0, 20}, {4, 0, 16}}, {{4, 0, 16}, {0, 0, 12}},
            {{0, 0, 12}, {4, 0, 8}},  {{4, 0, 8}, {0, 0, 4}}};
        REQUIRE(bars.size() == want.size());
        for (size_t i = 0; i < want.size(); ++i) {
            REQUIRE((bars[i].a - want[i].first).norm() < 1e-12);
            REQUIRE((bars[i].b - want[i].second).norm() < 1e-12);
            REQUIRE(bars[i].r == 0.5);
        }
    }
    SECTION("dynamic mode crosses only beyond two base radii") {
        // d = 4 is not > 2 * base_radius = 4: no crosses.
        REQUIRE(geom::truss_bars(mk(0, 0, 0), mk(1, 4, 0), prm).size() == 4);
        prm.mode = PillarConnectionMode::cross;
        const auto bars = geom::truss_bars(mk(0, 0, 0), mk(1, 4, 0), prm);
        REQUIRE(bars.size() == 8);
        REQUIRE((bars[1].a - Vec3d{4, 0, 20}).norm() < 1e-12);
        REQUIRE((bars[1].b - Vec3d{0, 0, 16}).norm() < 1e-12);
        prm.mode = PillarConnectionMode::zigzag;
        REQUIRE(geom::truss_bars(mk(0, 0, 0), mk(1, 8, 0), prm).size() == 2);
    }
    SECTION("no room for a full cross: centred on the room there is") {
        // q is 3 mm tall: one bar, centred, from z 6 to z 2.
        const auto bars = geom::truss_bars(mk(0, 0, 0), mk(1, 4, 0, 3.), prm);
        REQUIRE(bars.size() == 1);
        REQUIRE((bars[0].a - Vec3d{0, 0, 6}).norm() < 1e-12);
        REQUIRE((bars[0].b - Vec3d{4, 0, 2}).norm() < 1e-12);
    }
    SECTION("the bar radius is the thinner pillar's") {
        for (const Segment3 &b : geom::truss_bars(mk(0, 0, 0, 20., 1.), mk(1, 4, 0, 20., 0.4), prm))
            REQUIRE(b.r == 0.4);
    }
    SECTION("degenerate input gives no bars and ends") {
        REQUIRE(geom::truss_bars(mk(0, 0, 0), mk(1, 0, 0), prm).empty()); // same axis
        TrussParams flat = prm;
        flat.bridge_slope_rad = 0.;
        REQUIRE(geom::truss_bars(mk(0, 0, 0), mk(1, 4, 0), flat).empty());
        // Below the base: no usable length.
        REQUIRE(geom::truss_bars(mk(0, 0, 0, 0.5), mk(1, 4, 0), prm).empty());
    }
    SECTION("pillars standing on the model start above their feet") {
        for (const Segment3 &b : geom::truss_bars(mk(0, 0, 0, 10., 0.5, 25.),
                                                  mk(1, 4, 0, 10., 0.5, 25.), prm)) {
            REQUIRE(std::min(b.a.z(), b.b.z()) >= 25.);
            REQUIRE(std::max(b.a.z(), b.b.z()) <= 35.);
        }
    }
}

// ─── 3.4 candidates and order ────────────────────────────────────────────────

TEST_CASE("candidates go shortest first, ties by id", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(mk(3, 5, 20));
    pl.add_pillar(mk(2, 0, 20));
    pl.add_pillar(mk(1, 5, 0));
    pl.add_pillar(mk(0, 0, 0));

    // 0-1 and 2-3 are equally long; 0-2 and 1-3 are out of reach (20 mm).
    REQUIRE(strs(pl.run_greedy()) ==
            std::vector<std::string>{"0-1:accepted", "2-3:accepted"});
}

TEST_CASE("a near neighbour is braced before the one behind it", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(mk(0, 0, 0));
    pl.add_pillar(mk(1, 4, 0));
    pl.add_pillar(mk(2, 8, 0));

    REQUIRE(strs(pl.run_greedy()) ==
            std::vector<std::string>{"0-1:accepted", "1-2:accepted", "0-2:obstacle_pillar"});
}

TEST_CASE("a thicker pillar does not skip a thinner neighbour", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(mk(0, 0, 0, 20., 1.0));  // A
    pl.add_pillar(mk(1, 3, 0, 20., 0.5));  // M, thinner and nearest
    pl.add_pillar(mk(2, 0, 7, 20., 1.0));  // H, as thick and farther

    const auto ds = pl.run_greedy();
    REQUIRE(str(ds.front()) == "0-1:accepted");
    REQUIRE(pl.built()[size_t(ds.front().truss)].r == 0.5);
}

TEST_CASE("frozen pillars never pair with each other", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(frozen(mk(0, 0, 0), 0, 0));
    pl.add_pillar(frozen(mk(1, 5, 0), 0, 0));
    REQUIRE(pl.run_greedy().empty());

    // With one new pillar, only its candidates are tried.
    BracingPlanner one(config(), nullptr);
    one.add_pillar(frozen(mk(0, 0, 0), 0, 0));
    one.add_pillar(frozen(mk(1, 5, 0), 0, 0));
    one.add_pillar(mk(2, 2.5, 4));
    for (const Decision &d : one.run_greedy())
        REQUIRE((d.edge.a == 2 || d.edge.b == 2));
}

TEST_CASE("decisions do not depend on the order pillars are added", "[bracing_planner]")
{
    std::vector<PillarGeom> ps;
    for (int i = 0; i < 25; ++i)
        ps.push_back(mk(i, 3.1 * (i % 5) + 0.37 * (i / 5), 2.9 * (i / 5) + 0.21 * (i % 3),
                        12. + (i * 7) % 20));

    BracingPlanner fwd(config(), nullptr), back(config(), nullptr);
    for (const PillarGeom &p : ps) fwd.add_pillar(p);
    for (auto it = ps.rbegin(); it != ps.rend(); ++it) back.add_pillar(*it);

    const auto a = fwd.run_greedy(), b = back.run_greedy();
    REQUIRE(strs(a) == strs(b));
    REQUIRE(strs(fwd.run_rescue()) == strs(back.run_rescue()));
}

TEST_CASE("a zero-length candidate is too close", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(mk(0, 0, 0));
    pl.add_pillar(mk(1, 0, 0));
    REQUIRE(strs(pl.run_greedy()) == std::vector<std::string>{"0-1:too_close"});
}

TEST_CASE("too_close threshold at limit +- 1e-6", "[bracing_planner]")
{
    for (double d : {1. - DELTA, 1. + DELTA}) {
        BracingPlanner pl(config(), nullptr);
        pl.add_pillar(mk(0, 0, 0));
        pl.add_pillar(mk(1, d, 0));
        const Reason r = pl.run_greedy().front().reason;
        REQUIRE((r == Reason::too_close) == (d < 1.));
    }
}

// ─── counts ──────────────────────────────────────────────────────────────────

TEST_CASE("pairs always count, links keep the height rule", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(mk(0, 0, 0, 30.)); // A
    pl.add_pillar(mk(1, 4, 0, 10.)); // B
    REQUIRE(strs(pl.run_greedy()) == std::vector<std::string>{"0-1:accepted"});
    REQUIRE(pl.pillar(0).pairs == 1);
    REQUIRE(pl.pillar(1).pairs == 1);
    REQUIRE(pl.pillar(0).links == 0); // 10 / 30 is not over 0.5, A is not under 15
    REQUIRE(pl.pillar(1).links == 1); // B is under 15
}

TEST_CASE("a pillar with three pairs takes no more", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    PillarGeom a = mk(0, 0, 0);
    a.pairs = 3; a.links = 1;
    pl.add_pillar(a);
    pl.add_pillar(mk(1, 4, 0));
    REQUIRE(strs(pl.run_greedy()) == std::vector<std::string>{"0-1:pair_cap"});
}

TEST_CASE("a group whose bars all hit the model is not built", "[bracing_planner]")
{
    BracingPlanner pl(config(), [](const Vec3d &, const Vec3d &, double) { return false; });
    pl.add_pillar(mk(0, 0, 0));
    pl.add_pillar(mk(1, 4, 0));
    REQUIRE(strs(pl.run_greedy()) == std::vector<std::string>{"0-1:mesh_hit"});
    REQUIRE(pl.pillar(0).pairs == 0);
    REQUIRE(pl.pillar(0).links == 0);
    REQUIRE(pl.built().empty());
}

TEST_CASE("pillars too short for a bar give no_room", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(mk(0, 0, 0, 0.5));
    pl.add_pillar(mk(1, 4, 0));
    REQUIRE(strs(pl.run_greedy()) == std::vector<std::string>{"0-1:no_room"});
}

// ─── 3.5 plan rules ──────────────────────────────────────────────────────────

TEST_CASE("a full pillar blocks the line over it", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    PillarGeom b = mk(1, 4, 0);
    b.pairs = 3; b.links = 1;
    pl.add_pillar(mk(0, 0, 0));   // A
    pl.add_pillar(b);             // B, full
    pl.add_pillar(mk(2, 8, 0.5)); // C
    pl.add_pillar(mk(3, 4, 6));   // D

    const auto ds = pl.run_greedy();
    REQUIRE(find(ds, 0, 2)->reason == Reason::obstacle_pillar);
    REQUIRE(find(ds, 0, 3)->reason == Reason::accepted);
}

TEST_CASE("obstacle pillar threshold at limit +- 1e-6, at any height", "[bracing_planner]")
{
    for (double off : {LIMIT - DELTA, LIMIT + DELTA}) {
        BracingPlanner pl(config(), nullptr);
        PillarGeom m = mk(1, 4, off, 5., 0.5, 20.); // stands on the model, 20 mm up
        m.pairs = 3;
        pl.add_pillar(mk(0, 0, 0));
        pl.add_pillar(m);
        pl.add_pillar(mk(2, 8, 0));
        const Reason r = find(pl.run_greedy(), 0, 2)->reason;
        REQUIRE((r == Reason::obstacle_pillar) == (off < LIMIT));
    }
}

TEST_CASE("an X crossing is refused", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(mk(0, 0, 0)); // A
    pl.add_pillar(mk(1, 6, 6)); // B
    pl.add_pillar(mk(2, 0, 6)); // C
    pl.add_pillar(mk(3, 6, 0)); // D

    const auto ds = pl.run_greedy();
    REQUIRE(find(ds, 0, 1)->reason == Reason::accepted);    // A-B, first diagonal
    REQUIRE(find(ds, 2, 3)->reason == Reason::cross_truss); // C-D
}

namespace {

// Existing bracing A-B on two full frozen pillars, as a caller hands it in.
void existing_ab(BracingPlanner &pl, Vec2d a, Vec2d b, double z = 5.)
{
    pl.add_pillar(frozen(mk(100, a.x(), a.y()), 3));
    pl.add_pillar(frozen(mk(101, b.x(), b.y()), 3));
    pl.add_truss_bar(100, 101, seg({a.x(), a.y(), z + 2}, {b.x(), b.y(), z}));
}

} // namespace

TEST_CASE("a T against existing bracing is refused", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    existing_ab(pl, {0, 0}, {8, 0});
    pl.add_pillar(mk(0, 4, 5));
    pl.add_pillar(mk(1, 4, 0.1));
    REQUIRE(find(pl.run_greedy(), 0, 1)->reason == Reason::cross_truss);
}

TEST_CASE("parallel groups: plan gap at limit and +- 1e-6", "[bracing_planner]")
{
    for (double g : {LIMIT - DELTA, LIMIT, LIMIT + DELTA}) {
        BracingPlanner pl(config(), nullptr);
        existing_ab(pl, {0, 0}, {10, 0});
        pl.add_pillar(mk(0, 3, g));
        pl.add_pillar(mk(1, 7, g));
        const Reason r = find(pl.run_greedy(), 0, 1)->reason;
        CAPTURE(g);
        REQUIRE((r == Reason::cross_truss) == (g < LIMIT));
        if (g >= LIMIT) REQUIRE(r == Reason::accepted);
    }
}

TEST_CASE("collinear groups", "[bracing_planner]")
{
    SECTION("contained in existing bracing") {
        BracingPlanner pl(config(), nullptr);
        existing_ab(pl, {0, 0}, {10, 0});
        pl.add_pillar(mk(0, 3, 0));
        pl.add_pillar(mk(1, 7, 0));
        REQUIRE(find(pl.run_greedy(), 0, 1)->reason == Reason::cross_truss);
    }
    SECTION("overlapping: the end pillar in the way is met first") {
        BracingPlanner pl(config(), nullptr);
        existing_ab(pl, {0, 0}, {6, 0});
        pl.add_pillar(mk(0, 3, 0));
        pl.add_pillar(mk(1, 9, 0));
        REQUIRE(find(pl.run_greedy(), 0, 1)->reason == Reason::obstacle_pillar);
    }
    SECTION("apart") {
        BracingPlanner pl(config(), nullptr);
        existing_ab(pl, {0, 0}, {4, 0});
        pl.add_pillar(mk(0, 6, 0));
        pl.add_pillar(mk(1, 10, 0));
        REQUIRE(find(pl.run_greedy(), 0, 1)->reason == Reason::accepted);
    }
}

TEST_CASE("groups sharing a pillar are judged by angle", "[bracing_planner]")
{
    for (double deg : {19., 21., 20. - DELTA, 20. + DELTA, 45.}) {
        BracingPlanner pl(config(), nullptr);
        PillarGeom s = mk(0, 0, 0);
        s.pairs = 1;
        pl.add_pillar(s);
        pl.add_pillar(frozen(mk(1, 6, 0), 3));
        pl.add_truss_bar(0, 1, seg({0, 0, 15}, {6, 0, 9}));
        const Vec2d c = polar(deg, 7.);
        pl.add_pillar(mk(2, c.x(), c.y()));

        const Reason r = find(pl.run_greedy(), 0, 2)->reason;
        CAPTURE(deg);
        REQUIRE((r == Reason::shared_angle) == (deg < 20.));
        if (deg >= 20.) REQUIRE(r == Reason::accepted);
    }
}

// ─── 3.5 bar by bar ──────────────────────────────────────────────────────────

namespace {

// P(0,0)-Q(4,0), 20 mm: four bars crossing x = 2 at z 18, 14, 10 and 6, all
// at 45 degrees in the XZ plane. A horizontal bar along Y at x = 2 meets the
// first of them at 3D distance (z - 18) * kappa, kappa = cos 45.
BracingPlanner pq_with_bar_over_first(double gap3d, long owner_a = -1, long owner_b = -1)
{
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(mk(0, 0, 0));
    pl.add_pillar(mk(1, 4, 0));
    const double z = 18. + gap3d / std::cos(PI / 4);
    pl.add_obstacle_bar({seg({2, -3, z}, {2, 3, z}), owner_a, owner_b});
    return pl;
}

} // namespace

TEST_CASE("a bar too near another bar in 3D is left out", "[bracing_planner]")
{
    SECTION("1.5 mm apart: built") {
        auto pl = pq_with_bar_over_first(1.5);
        const auto d = pl.run_greedy().front();
        REQUIRE(d.reason == Reason::accepted);
        REQUIRE(d.bars_built == 4);
    }
    SECTION("1.0 mm apart: that bar only is left out") {
        auto pl = pq_with_bar_over_first(1.0);
        const auto d = pl.run_greedy().front();
        REQUIRE(d.reason == Reason::accepted);
        REQUIRE(d.bars_built == 3);
        for (const Segment3 &b : pl.built()[size_t(d.truss)].bars)
            REQUIRE(std::max(b.a.z(), b.b.z()) < 17.);
    }
    SECTION("threshold +- 1e-6") {
        REQUIRE(pq_with_bar_over_first(LIMIT - DELTA).run_greedy().front().bars_built == 3);
        REQUIRE(pq_with_bar_over_first(LIMIT + DELTA).run_greedy().front().bars_built == 4);
    }
    SECTION("a bar ending on one of the pair's pillars is not exempt away from it") {
        // At x = 2 the contact is 2 mm from either axis, outside R = 1.7
        // (sla-head-bridge-clearance): ending on the pillar exempts nothing
        // there. Within R, see "a bar is exempt only within R of the pillar
        // it shares".
        REQUIRE(pq_with_bar_over_first(0.3, -1, 0).run_greedy().front().bars_built == 3);
        REQUIRE(pq_with_bar_over_first(0.3, 1, -1).run_greedy().front().bars_built == 3);
        REQUIRE(pq_with_bar_over_first(0.3, 7, 8).run_greedy().front().bars_built == 3);
    }
    SECTION("every bar blocked: z_clear_fail, nothing counted") {
        BracingPlanner pl(config(), nullptr);
        pl.add_pillar(mk(0, 0, 0));
        pl.add_pillar(mk(1, 4, 0));
        for (double z : {18., 14., 10., 6.})
            pl.add_obstacle_bar({seg({2, -3, z + 0.5}, {2, 3, z + 0.5}), -1, -1});
        const auto d = pl.run_greedy().front();
        REQUIRE(d.reason == Reason::z_clear_fail);
        REQUIRE(d.bars_built == 0);
        REQUIRE(pl.pillar(0).pairs == 0);
    }
    SECTION("z_clear_fail wins over the model when both stop bars") {
        BracingPlanner pl(config(), [](const Vec3d &a, const Vec3d &b, double) {
            return std::max(a.z(), b.z()) > 17.; // only the top bar would clear the model
        });
        pl.add_pillar(mk(0, 0, 0));
        pl.add_pillar(mk(1, 4, 0));
        pl.add_obstacle_bar({seg({2, -3, 18.5}, {2, 3, 18.5}), -1, -1});
        REQUIRE(pl.run_greedy().front().reason == Reason::z_clear_fail);
    }
}

// ─── 3.6 rescue ──────────────────────────────────────────────────────────────

namespace {

// P(4,-4.5)-Q(4,4.5) crosses existing bracing X(0,0)-Y(8,0) in plan view; X
// and Y are full, so P-Q is the only candidate. `stack` adds horizontal X-Y
// bars every 1 mm from z 30 to z 50, which no bar of P-Q can clear in 3D.
BracingPlanner crossing_rescue(double pq_foot, bool stack)
{
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(frozen(mk(100, 0, 0, 40.), 3));
    pl.add_pillar(frozen(mk(101, 8, 0, 40.), 3));
    pl.add_truss_bar(100, 101, seg({0, 0, 7}, {8, 0, 3}));
    if (stack)
        for (int z = 30; z <= 50; ++z)
            pl.add_truss_bar(100, 101, seg({0, 0, double(z)}, {8, 0, double(z)}));
    pl.add_pillar(mk(0, 4, -4.5, 16., 0.5, pq_foot)); // P, lonely and tall
    pl.add_pillar(mk(1, 4, 4.5, 16., 0.5, pq_foot));  // Q
    return pl;
}

} // namespace

TEST_CASE("rescue: braced when the bars clear in 3D", "[bracing_planner]")
{
    auto pl = crossing_rescue(30., false); // P-Q bars all above z 31
    REQUIRE(find(pl.run_greedy(), 0, 1)->reason == Reason::cross_truss);

    // Every candidate of a lonely pillar is tried again; the full ones are
    // refused again for the same reason.
    const auto rs = pl.run_rescue();
    REQUIRE(find(rs, 0, 100)->reason == Reason::pair_cap);
    const Decision *pq = find(rs, 0, 1);
    REQUIRE(pq->reason == Reason::accepted);
    REQUIRE(pq->rescue);
    REQUIRE(pl.built()[size_t(pq->truss)].rescued);
    REQUIRE(pl.pillar(0).links == 1); // 16 / 16 over 0.5
    REQUIRE(pl.pillar(0).pairs == 1);
}

TEST_CASE("rescue: refused when no bar clears in 3D", "[bracing_planner]")
{
    auto pl = crossing_rescue(30., true);
    REQUIRE(find(pl.run_greedy(), 0, 1)->reason == Reason::cross_truss);
    const auto rs = pl.run_rescue();
    REQUIRE(find(rs, 0, 1)->reason == Reason::z_clear_fail);
    for (const Decision &d : rs) REQUIRE(d.reason != Reason::accepted);
    REQUIRE(pl.pillar(0).links == 0);
}

TEST_CASE("rescue does not relax the obstacle rule", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    PillarGeom r = mk(2, 4, 0.2);
    r.pairs = 3;
    pl.add_pillar(mk(0, 0, 0, 30.)); // P
    pl.add_pillar(mk(1, 8, 0, 30.)); // Q
    pl.add_pillar(r);                // R, full, in the way
    REQUIRE(find(pl.run_greedy(), 0, 1)->reason == Reason::obstacle_pillar);
    REQUIRE(find(pl.run_rescue(), 0, 1)->reason == Reason::obstacle_pillar);
}

TEST_CASE("rescue skips pillars that are not lonely and tall", "[bracing_planner]")
{
    // 15 mm is not over the rescue height.
    BracingPlanner low(config(), nullptr);
    low.add_pillar(frozen(mk(100, 0, 0, 40.), 3));
    low.add_pillar(frozen(mk(101, 8, 0, 40.), 3));
    low.add_truss_bar(100, 101, seg({0, 0, 7}, {8, 0, 3}));
    low.add_pillar(mk(0, 4, -4.5, 15., 0.5, 30.));
    low.add_pillar(mk(1, 4, 4.5, 15., 0.5, 30.));
    low.run_greedy();
    REQUIRE(low.run_rescue().empty());
}

TEST_CASE("a rescued pillar is not rescued again", "[bracing_planner]")
{
    // P has two candidates over existing bracing; once the first gives it a
    // link, the second is not tried in its name.
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(frozen(mk(100, -2, 0, 40.), 3));
    pl.add_pillar(frozen(mk(101, 10, 0, 40.), 3));
    pl.add_truss_bar(100, 101, seg({-2, 0, 7}, {10, 0, 3}));
    pl.add_pillar(mk(0, 4, -4, 30., 0.5, 0.)); // P
    pl.add_pillar(mk(1, 1, 4, 20., 0.5, 0.));  // Q1, not a target itself
    pl.add_pillar(mk(2, 7.5, 4, 20., 0.5, 0.)); // Q2, farther from P
    const auto g = pl.run_greedy();
    REQUIRE(find(g, 0, 1)->reason == Reason::cross_truss);
    REQUIRE(find(g, 0, 2)->reason == Reason::cross_truss);

    const auto rs = pl.run_rescue();
    REQUIRE(find(rs, 0, 1)->reason == Reason::accepted);
    REQUIRE(pl.pillar(0).links == 1);
    REQUIRE_FALSE(find(rs, 0, 2)); // not tried: P is no longer lonely
}

// ─── auxiliary pillars ───────────────────────────────────────────────────────

TEST_CASE("try_attach changes nothing, attach does", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(mk(0, 0, 0, 30.));
    pl.run_greedy();

    const PillarGeom prop = mk(1, 4, 0, 26.);
    const Decision t = pl.try_attach(0, prop);
    REQUIRE(t.reason == Reason::accepted);
    REQUIRE(t.bars_built > 0);
    REQUIRE(pl.built().empty());
    REQUIRE(pl.pillar(0).pairs == 0);

    pl.add_pillar(prop);
    const Decision a = pl.attach(0, 1);
    REQUIRE(a.reason == Reason::accepted);
    REQUIRE(a.bars_built == t.bars_built);
    REQUIRE(pl.pillar(0).pairs == 1);
    REQUIRE(pl.pillar(1).pairs == 1);

    REQUIRE(pl.try_attach(0, mk(2, 40, 0)).reason == Reason::out_of_range);
}

TEST_CASE("only the new pillars are run again, never from a frozen one", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(frozen(mk(0, 0, 0), 0, 0));
    pl.add_pillar(frozen(mk(1, 5, 0), 0, 0));
    pl.run_greedy();

    pl.add_pillar(mk(2, 2.5, 4));
    const auto ds = pl.run_greedy_for({2, 0});
    REQUIRE_FALSE(ds.empty());
    for (const Decision &d : ds) REQUIRE((d.edge.a == 2 || d.edge.b == 2));
    REQUIRE_FALSE(find(ds, 0, 1));
}

TEST_CASE("a pillar that is not braceable is only in the way", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(mk(0, 0, 0));
    pl.add_pillar(mk(1, 8, 0));
    PillarGeom own = mk(2, 4, 0.3, 5., 0.5, 15.); // a head's own short pillar
    own.braceable = false;
    pl.add_pillar(own);
    PillarGeom far = mk(3, 0, 5);
    pl.add_pillar(far);
    PillarGeom lone = mk(4, 30, 0, 30.);
    lone.braceable = false;
    pl.add_pillar(lone);

    const auto ds = pl.run_greedy();
    for (const Decision &d : ds) REQUIRE((d.edge.a != 2 && d.edge.b != 2));
    REQUIRE(find(ds, 0, 1)->reason == Reason::obstacle_pillar);
    REQUIRE(find(ds, 0, 3)->reason == Reason::accepted);
    // Never rescued either, however lonely and tall.
    for (const Decision &d : pl.run_rescue()) REQUIRE((d.edge.a != 4 && d.edge.b != 4));
}

// ─── 4.7 clusters of hugging pillars ─────────────────────────────────────────

TEST_CASE("hugging pillars are one cluster, by connection", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(mk(0, 0, 0));
    pl.add_pillar(mk(1, 2.2, 0));
    pl.add_pillar(mk(2, 1.1, 0)); // joins 0 and 1, 2.2 apart, through itself
    REQUIRE(pl.cluster_of(0) == pl.cluster_of(1));
    REQUIRE(pl.cluster_of(0) == pl.cluster_of(2));

    // Exactly r1 + r2 + c apart is not hugging; a hair closer is.
    pl.add_pillar(mk(3, 0, 10));
    pl.add_pillar(mk(4, LIMIT, 10));
    pl.add_pillar(mk(5, 0, 20));
    pl.add_pillar(mk(6, LIMIT - DELTA, 20));
    REQUIRE(pl.cluster_of(3) != pl.cluster_of(4));
    REQUIRE(pl.cluster_of(5) == pl.cluster_of(6));
    REQUIRE(pl.cluster_of(0) != pl.cluster_of(3));
    REQUIRE(pl.cluster_of(99) == -1);
}

TEST_CASE("hugging pillars are not in each other's way", "[bracing_planner]")
{
    // A-X passes 0.9 from A', closer than 1.2: a third pillar would stop it.
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(mk(0, 0, 0));   // A
    pl.add_pillar(mk(1, 0.9, 0)); // A', hugging A
    pl.add_pillar(mk(2, 0, 6));   // X
    REQUIRE(find(pl.run_greedy(), 0, 2)->reason == Reason::accepted);
}

TEST_CASE("groups leaving one cluster are judged by angle", "[bracing_planner]")
{
    // A-X exists (0 deg from A); A' hugs A. A'-Y leaves the same cluster.
    auto run = [](double deg) {
        BracingPlanner pl(config(), nullptr);
        pl.add_pillar(frozen(mk(0, 0, 0), 1)); // A
        pl.add_pillar(frozen(mk(2, 6, 0), 3)); // X, full
        pl.add_truss_bar(0, 2, seg({0, 0, 15}, {6, 0, 9}));
        pl.add_pillar(mk(1, 0.9, 0));          // A'
        const Vec2d y = Vec2d{0.9, 0} + polar(deg, 3.);
        pl.add_pillar(mk(3, y.x(), y.y()));    // Y
        return find(pl.run_greedy(), 1, 3)->reason;
    };
    // Without the cluster both would be obstacle_pillar: A is 0.9 from A'.
    REQUIRE(run(45.) == Reason::accepted);
    REQUIRE(run(10.) == Reason::shared_angle);
}

// ─── 4.8 auxiliary pillars and the pair cap ──────────────────────────────────

TEST_CASE("a prop is braced to a pillar whose pairs are full", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    PillarGeom p = mk(0, 0, 0, 30.);
    p.pairs = 3; // all to short neighbours: no link
    pl.add_pillar(p);
    pl.run_greedy();

    const PillarGeom prop = mk(1, 4, 0, 26.);
    REQUIRE(pl.try_attach(0, prop).reason == Reason::accepted);
    pl.add_pillar(prop);
    REQUIRE(pl.attach(0, 1).reason == Reason::accepted);
    REQUIRE(pl.pillar(0).pairs == 4);
    REQUIRE(pl.pillar(0).links == 1);
}

// ─── 4.9 safety net ──────────────────────────────────────────────────────────

namespace {

PillarGeom counted(PillarGeom p, unsigned pairs, unsigned links)
{
    p.pairs = pairs; p.links = links;
    return p;
}

} // namespace

TEST_CASE("safety net: braced past a neighbour's full pairs", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(mk(0, 0, 0, 30.));                    // P, lonely and tall
    pl.add_pillar(counted(mk(1, 4, 0, 30.), 3, 2));     // Q
    REQUIRE(find(pl.run_greedy(), 0, 1)->reason == Reason::pair_cap);
    REQUIRE(find(pl.run_rescue(), 0, 1)->reason == Reason::pair_cap);

    const auto fb = pl.run_fallback();
    const Decision *pq = find(fb, 0, 1);
    REQUIRE(pq->reason == Reason::accepted);
    REQUIRE(pq->fallback);
    REQUIRE(pl.built()[size_t(pq->truss)].fallback);
    REQUIRE_FALSE(pl.built()[size_t(pq->truss)].rescued);
    REQUIRE(pl.pillar(0).links == 1);
    REQUIRE(pl.pillar(1).pairs == 4);
    REQUIRE(pl.pillar(1).links == 3);
    REQUIRE(pl.run_fallback().empty()); // nobody is lonely any more
}

TEST_CASE("safety net: a neighbour with three links is not braced", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(mk(0, 0, 0, 30.));                // P
    pl.add_pillar(counted(mk(1, 4, 0, 30.), 3, 3)); // Q, links full
    pl.add_pillar(counted(mk(2, 0, 5, 30.), 3, 1)); // R, farther
    pl.run_greedy();
    pl.run_rescue();

    const auto fb = pl.run_fallback();
    REQUIRE(find(fb, 0, 1)->reason == Reason::pair_cap);
    REQUIRE(find(fb, 0, 2)->reason == Reason::accepted);
    REQUIRE(pl.pillar(1).pairs == 3);
    REQUIRE(pl.pillar(0).links == 1);
}

TEST_CASE("safety net: a short neighbour is no candidate", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(mk(0, 0, 0, 30.));                // P
    pl.add_pillar(counted(mk(1, 4, 0, 10.), 3, 1)); // 10 / 30 gives P no link
    pl.run_greedy();
    pl.run_rescue();
    REQUIRE(pl.run_fallback().empty());
    REQUIRE(pl.pillar(0).links == 0);
}

TEST_CASE("safety net: still not through a pillar", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(mk(0, 0, 0, 30.));                  // P
    pl.add_pillar(counted(mk(1, 8, 0, 30.), 3, 1));   // Q
    pl.add_pillar(counted(mk(2, 4, 0.2, 30.), 3, 3)); // R, in the way, links full
    pl.run_greedy();
    pl.run_rescue();

    const auto fb = pl.run_fallback();
    REQUIRE(find(fb, 0, 1)->reason == Reason::obstacle_pillar);
    REQUIRE(find(fb, 0, 2)->reason == Reason::pair_cap);
    REQUIRE(pl.pillar(0).links == 0);
}

// ─── sla-head-bridge-clearance: the joint rule, one group per pair ───────────

namespace {

// P(0,0)-Q(4,0), 20 mm, as above: the bars meet P's axis at z 20, 12, 12 and
// 4. `extra` is one more pillar, in P's cluster or not; the bar is judged
// against the group with the owners given.
std::vector<Decision> pq_with(const ObstacleBar &bar, const PillarGeom *extra = nullptr)
{
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(mk(0, 0, 0));
    pl.add_pillar(mk(1, 4, 0));
    if (extra) pl.add_pillar(*extra);
    pl.add_obstacle_bar(bar);
    return pl.run_greedy();
}

// The decision on pair (a, b), which must be among `ds`.
Decision decision(const std::vector<Decision> &ds, long a, long b)
{
    const Decision *d = find(ds, a, b);
    REQUIRE(d != nullptr);
    return *d;
}

// A head's own short pillar hugging P: in P's cluster, never braced.
PillarGeom member(double x, double y, double r)
{
    PillarGeom m = mk(2, x, y, 5., r, 10.);
    m.braceable = false;
    return m;
}

} // namespace

TEST_CASE("a bar is exempt only within R of the pillar it shares", "[bracing_planner]")
{
    // A head bar ending on P's axis at z 13 comes 0.58 from the bar reaching
    // P's axis at z 12, 0.67 from that axis: within R = 1.7.
    const Segment3 head = seg({2, -2, 13}, {0, 0, 13});

    SECTION("ending on P: accepted, every bar") {
        const Decision d = decision(pq_with({head, -1, 0}), 0, 1);
        REQUIRE(d.reason == Reason::accepted);
        REQUIRE(d.bars_built == 4);
    }
    SECTION("ending on no pillar of the group: judged whole") {
        REQUIRE(decision(pq_with({head, -1, -1}), 0, 1).bars_built == 2);
        REQUIRE(decision(pq_with({head, -1, 7}), 0, 1).bars_built == 2);
    }
    SECTION("named as ending on Q: clipped at Q only, where it is not") {
        REQUIRE(decision(pq_with({head, 1, -1}), 0, 1).bars_built == 2);
    }
    SECTION("ending on P, but in the way of every bar away from it: z_clear_fail") {
        // Bars of the pair 2.8 mm from P's axis, outside R (the spec's
        // scenario, here with two pillars 8 mm tall: two crossing bars).
        BracingPlanner pl(config(), nullptr);
        pl.add_pillar(mk(0, 0, 0, 8.));
        pl.add_pillar(mk(1, 6, 0, 8.));
        pl.add_obstacle_bar({seg({3, 0.6, 5}, {0, 0, 5}), -1, 0});
        const Decision d = pl.run_greedy().front();
        REQUIRE(d.reason == Reason::z_clear_fail);
        REQUIRE(d.bars_built == 0);
    }
}

TEST_CASE("a bar ending on a cluster member is clipped by that member", "[bracing_planner]")
{
    SECTION("by its own radius") {
        // M, r 1, hugs P 1.5 away. A bar ending on M's axis lies wholly within
        // R_M = 1 + 0.5 + 0.5 + 0.2 = 2.2 of it: exempt. With P's radius, 0.5,
        // R would be 1.7 and a bar of the pair would be left out.
        const PillarGeom m = member(0, -1.5, 1.);
        const ObstacleBar bar{seg({2, -1, 13}, {0, -1.5, 13}), -1, 2};
        const Decision d = decision(pq_with(bar, &m), 0, 1);
        REQUIRE(d.reason == Reason::accepted);
        REQUIRE(d.bars_built == 4);
        // The same bar, ending on no pillar.
        REQUIRE(decision(pq_with({bar.seg, -1, -1}, &m), 0, 1).bars_built == 3);
    }
    SECTION("by its own axis") {
        // M, r 1, hugs P 1.5 behind it. The bar ending on M's axis comes 0.88
        // from a bar of the pair 2.27 from M's axis, outside R_M = 2.2; P's
        // axis would have clipped it away.
        const PillarGeom m = member(-1.5, 0, 1.);
        const ObstacleBar bar{seg({1, -1, 11}, {-1.5, 0, 11}), -1, 2};
        const Decision d = decision(pq_with(bar, &m), 0, 1);
        REQUIRE(d.reason == Reason::accepted);
        REQUIRE(d.bars_built == 3);
    }
    SECTION("by the larger of its radii") {
        // M, r 0.75, hugs P 1.4 away. With r_joint 1 (r_end above r_start)
        // R_M is 2.2 and holds the whole bar; by r alone it is 1.95, and a bar
        // of the pair is left out.
        PillarGeom m = member(0, -1.4, 0.75);
        const ObstacleBar bar{seg({2, -0.5, 13}, {0, -1.4, 13}), -1, 2};
        REQUIRE(decision(pq_with(bar, &m), 0, 1).bars_built == 3);
        m.r_joint = 1.;
        REQUIRE(decision(pq_with(bar, &m), 0, 1).bars_built == 4);
    }
}

TEST_CASE("attach refuses a pair braced already", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    PillarGeom p = mk(0, 0, 0, 30.);
    p.pairs = 3;
    pl.add_pillar(p);
    pl.run_greedy();
    pl.add_pillar(mk(1, 4, 0, 26.));

    const Decision first = pl.attach(0, 1);
    REQUIRE(first.reason == Reason::accepted);
    const size_t groups = pl.built().size();

    for (const auto &[a, b] : std::vector<std::pair<long, long>>{{0, 1}, {1, 0}}) {
        const Decision again = pl.attach(a, b);
        REQUIRE(again.reason == Reason::duplicate_pair);
        REQUIRE(std::string(reason_name(again.reason)) == "duplicate_pair");
        REQUIRE(again.edge.a == 0);
        REQUIRE(again.edge.b == 1);
        REQUIRE(again.bars_built == 0);
        REQUIRE(again.truss == -1);
    }
    REQUIRE(pl.built().size() == groups);
    REQUIRE(pl.pillar(0).pairs == 4);
    REQUIRE(pl.pillar(1).pairs == 1);
}

TEST_CASE("attach refuses a pair the greedy pass braced", "[bracing_planner]")
{
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(mk(0, 0, 0));
    pl.add_pillar(mk(1, 4, 0));
    REQUIRE(pl.run_greedy().front().reason == Reason::accepted);
    REQUIRE(pl.attach(1, 0).reason == Reason::duplicate_pair);
    REQUIRE(pl.built().size() == 1);
}

TEST_CASE("run_greedy_for never offers a pair braced already", "[bracing_planner]")
{
    // A prop braced to its pillar by attach(), then run again with a second
    // prop: neither pair with the pillar comes up twice.
    BracingPlanner pl(config(), nullptr);
    pl.add_pillar(mk(0, 0, 0, 30.));
    pl.run_greedy();
    pl.add_pillar(mk(1, 4, 0, 26.));
    pl.add_pillar(mk(2, 0, 4, 26.));
    REQUIRE(pl.attach(0, 1).reason == Reason::accepted);
    REQUIRE(pl.attach(0, 2).reason == Reason::accepted);
    REQUIRE(pl.attach(1, 2).reason == Reason::accepted);

    REQUIRE(pl.run_greedy_for({1, 2}).empty());

    std::set<std::pair<long, long>> pairs;
    for (const BuiltTruss &t : pl.built()) REQUIRE(pairs.insert({t.a, t.b}).second);
    REQUIRE(pairs.size() == 3);
}

// ─── the engine stage (tasks 4.5 and 4.6) ────────────────────────────────────

#include "sla_test_utils.hpp"
#include "libslic3r/SLA/DefaultSupportTree.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"

TEST_CASE("frozen pillars only take bracing from new ones, and report pairs",
          "[bracing_planner]")
{
    // A cube lifted 10 mm, with a row of pillars from an earlier generation
    // standing along one side of it.
    sla::SupportTreeConfig cfg;
    cfg.object_elevation_mm = 10.;
    TriangleMesh mesh = load_model("20mm_cube.obj");
    const auto bb = mesh.bounding_box();
    const double gnd = bb.min.z() - cfg.object_elevation_mm;

    auto slicegrid = grid(float(gnd), float(bb.max.z()), 0.05f);
    auto slices    = slice_mesh_ex(mesh.its, slicegrid, CLOSING_RADIUS);
    sla::SupportableMesh sm{mesh.its, {}, cfg};
    sla::SupportPointGeneratorConfig autogencfg;
    auto gen = sla::prepare_generator_data(std::move(slices), slicegrid);
    sm.pts = sla::move_on_mesh_surface(sla::generate_support_points(gen, autogencfg), sm.emesh,
                                       (slicegrid[1] - slicegrid[0]) +
                                           std::numeric_limits<float>::epsilon());
    sm.zoffset = bb.min.z();
    REQUIRE_FALSE(sm.pts.empty());

    for (long i = 0; i < 4; ++i) {
        sla::PriorPillar pp;
        pp.id       = 500 + i;
        pp.endpoint = {bb.min.x() - 3., bb.min.y() + 2. + 5. * double(i), gnd};
        pp.height   = 25.;
        pp.r_start  = pp.r_end = 0.5;
        pp.links = pp.pairs = i == 0 ? 1u : 0u; // one of them already braced once
        sm.prior.push_back(pp);
    }

    sla::SupportTreeBuilder builder;
    sla::PriorAttachments attached;
    sla::DefaultSupportTree::execute(builder, sm, &attached);

    const auto &pillars = builder.pillars();
    std::map<long, unsigned> groups; // frozen builder id -> distinct partners
    std::set<std::pair<long, long>> pairs;
    for (const sla::Bridge &b : builder.crossbridges()) {
        REQUIRE(b.kind == sla::BridgeKind::truss);
        const bool fa = pillars[size_t(b.owner_a)].frozen, fb = pillars[size_t(b.owner_b)].frozen;
        REQUIRE_FALSE((fa && fb)); // never frozen to frozen
        pairs.insert({std::min(b.owner_a, b.owner_b), std::max(b.owner_a, b.owner_b)});
    }
    for (const auto &[a, b] : pairs) {
        if (pillars[size_t(a)].frozen) ++groups[a];
        if (pillars[size_t(b)].frozen) ++groups[b];
    }

    // No prop was grown for a frozen pillar.
    for (const sla::Pillar &p : pillars)
        if (p.props_for >= 0) REQUIRE_FALSE(pillars[size_t(p.props_for)].frozen);

    // Every frozen pillar braced to is reported, with its counts after the run.
    REQUIRE_FALSE(attached.empty()); // the new supports did reach the old row
    REQUIRE(attached.size() == groups.size());
    for (const sla::PriorAttachment &pa : attached) {
        long fid = -1;
        for (const sla::Pillar &p : pillars)
            if (p.frozen && fid < 0 && 500 + (&p - pillars.data()) == pa.prior_id) fid = p.id;
        REQUIRE(fid >= 0);
        const unsigned before = pa.prior_id == 500 ? 1u : 0u;
        REQUIRE(pa.pairs == before + groups[fid]);
        REQUIRE(pa.pairs == pillars[size_t(fid)].pairs);
        REQUIRE(pa.links == pillars[size_t(fid)].links);
        REQUIRE(pa.pairs >= pa.links);
    }
}

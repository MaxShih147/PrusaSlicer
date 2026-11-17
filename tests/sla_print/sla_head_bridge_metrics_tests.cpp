// The head route metrics (openspec change sla-head-bridge-clearance, task
// 6.4), each count on a small tree laid out by hand. Heads point straight
// down with r_back 0.5 unless said otherwise: the junction is 1.7 mm under
// the head's point, and two elements touch below r1 + r2 + 0.2 mm.

#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include <nlohmann/json.hpp>

#include "libslic3r/SLA/RoutePlan.hpp"
#include "libslic3r/SLA/SupportTreeBuilder.hpp"

#include "sla_head_bridge_metrics.hpp"

using namespace Slic3r;
using namespace Slic3r::sla;

namespace {

// A head whose junction is at `jp`.
void head(SupportTreeBuilder &b, unsigned id, const Vec3d &jp, double r_back = 0.5)
{
    // fullwidth - r_back = 2 * 0.2 + 1 + 2 * r_back - 0.2 - r_back
    const double drop = 1.2 + r_back;
    b.add_head(id, Head{r_back, 0.2, 1., 0.2, DOWN, jp + Vec3d{0., 0., drop}});
    REQUIRE((b.head(id).junction_point() - jp).norm() < 1e-9);
}

// A pillar on the ground at (x, y) whose top is at `top`.
long pillar(SupportTreeBuilder &b, double x, double y, double top, double r = 0.5)
{
    return b.add_pillar(Vec3d{x, y, 0.}, top, r);
}

// What a route record names: `head` kept too close to `with_head`.
RouteRecord record(long head, long with_head, RouteLevel level = RouteLevel::L1)
{
    RouteRecord rec;
    rec.head  = head;
    rec.level = level;
    clearance::Contact ct;
    ct.with_head = with_head;
    rec.contacts.push_back(ct);
    return rec;
}

HeadBridgeMetrics measure(const SupportTreeBuilder &b, const ClearanceReport &rep = {})
{
    const HeadBridgeMetrics m = head_bridge_metrics(b, rep);
    for (const std::string &s : m.unrecorded_list) UNSCOPED_INFO(s);
    REQUIRE(m.unassigned == 0);
    return m;
}

} // namespace

TEST_CASE("two heads' bars crossing are A, recorded when a record names them",
          "[head_bridge_metrics]")
{
    SupportTreeBuilder b;
    const long p0 = pillar(b, 5., 3., 18.3), p1 = pillar(b, 8., 0., 18.3);
    head(b, 0, Vec3d{5., -3., 18.3});
    head(b, 1, Vec3d{2., 0., 18.3});
    b.add_bridge_to_pillar(0, Vec3d{5., 3., 18.3}, p0);
    b.add_bridge_to_pillar(1, Vec3d{8., 0., 18.3}, p1);

    SECTION("no record") {
        const HeadBridgeMetrics m = measure(b);
        REQUIRE(m.heads_valid == 2);
        REQUIRE(m.head_bars == 2);
        REQUIRE(m.A == 1);
        REQUIRE(m.C_pillar + m.C_head + m.revC_head + m.revC_aux + m.A_prop + m.B == 0);
        REQUIRE(m.recorded == 0);
        REQUIRE(m.unrecorded == 1);
        REQUIRE(m.unrecorded_list.size() == 1);
        REQUIRE(m.D == 0);
    }
    SECTION("head 1 was routed at L1 against head 0") {
        ClearanceReport rep;
        rep.routes.push_back(record(1, 0));
        rep.l1 = 1;
        const HeadBridgeMetrics m = measure(b, rep);
        REQUIRE(m.A == 1);
        REQUIRE(m.recorded == 1);
        REQUIRE(m.unrecorded == 0);
        REQUIRE(m.l1 == 1);
    }
    SECTION("a record about another pair does not count") {
        ClearanceReport rep;
        rep.routes.push_back(record(1, 7, RouteLevel::L2));
        const HeadBridgeMetrics m = measure(b, rep);
        REQUIRE(m.recorded == 0);
        REQUIRE(m.unrecorded == 1);
    }
}

TEST_CASE("a bar through a third pillar is C_pillar, through a head C_head",
          "[head_bridge_metrics]")
{
    SupportTreeBuilder b;
    const long p1 = pillar(b, 8., 0., 18.3);
    head(b, 1, Vec3d{2., 0., 18.3});
    b.add_bridge_to_pillar(1, Vec3d{8., 0., 18.3}, p1);

    SECTION("pillar") {
        pillar(b, 5., 0.5, 30.); // 0.5 mm from the bar's axis
        const HeadBridgeMetrics m = measure(b);
        REQUIRE(m.C_pillar == 1);
        REQUIRE(m.C_head + m.A + m.revC_head == 0);
        REQUIRE(m.unrecorded == 1);
    }
    SECTION("head") {
        head(b, 2, Vec3d{5., 0.3, 17.8}); // the bar runs through its cone
        const HeadBridgeMetrics m = measure(b);
        REQUIRE(m.C_head == 1);
        REQUIRE(m.C_pillar + m.A + m.revC_head == 0);
        REQUIRE(m.unrecorded == 1);
    }
    SECTION("its own pillar and its own head are not counted") {
        const HeadBridgeMetrics m = measure(b);
        REQUIRE(m.C_pillar + m.C_head + m.unrecorded == 0);
    }
}

TEST_CASE("a pillar beside a head is reverse C, native or not", "[head_bridge_metrics]")
{
    // Head 0 grows its own pillar straight down from its junction.
    SupportTreeBuilder b;
    head(b, 0, Vec3d{0., 0., 18.3});
    const long own = b.add_pillar(long(0), 18.3);

    SECTION("lower down: not support point spacing") {
        head(b, 1, Vec3d{0.9, 0., 8.3});
        const HeadBridgeMetrics m = measure(b);
        REQUIRE(m.revC_head == 1);
        REQUIRE(m.revC_head_net == 1);
        REQUIRE(m.revC_head_native == 0);
        REQUIRE(m.unrecorded == 1);

        // Recorded as a centre pillar contact: the pillar is head 0's.
        ClearanceReport rep;
        CentroidRecord  cr;
        cr.pillar            = own;
        cr.contact.with_head = 1;
        rep.centroid.push_back(cr);
        const HeadBridgeMetrics r = measure(b, rep);
        REQUIRE(r.recorded == 1);
        REQUIRE(r.unrecorded == 0);
        REQUIRE(r.centroid_recorded == 1);
    }
    SECTION("beside the head that grew it: support point spacing") {
        head(b, 1, Vec3d{0.9, 0., 18.3}); // 0.9 mm from head 0, short of 1.2
        const HeadBridgeMetrics m = measure(b);
        REQUIRE(m.revC_head == 1);
        REQUIRE(m.revC_head_native == 1);
        REQUIRE(m.revC_head_net == 0);
        REQUIRE(m.recorded + m.unrecorded == 0);
    }
}

TEST_CASE("auxiliary pillars and prop bars are A_prop and revC_aux, never recorded",
          "[head_bridge_metrics]")
{
    SupportTreeBuilder b;
    const long p0  = pillar(b, 0., 0., 20.);
    const long aux = pillar(b, 6., 0., 15.);
    b.pillar(aux).props_for = p0;
    b.add_bridge_between(aux, Vec3d{0., 0., 15.}, Vec3d{6., 0., 15.}, 0.5, BridgeKind::prop, p0);

    // A head bar 0.3 mm over the prop bar.
    const long p2 = pillar(b, 3., 3., 15.3);
    head(b, 0, Vec3d{3., -3., 15.3});
    b.add_bridge_to_pillar(0, Vec3d{3., 3., 15.3}, p2);
    // A head beside the auxiliary pillar.
    head(b, 1, Vec3d{6.9, 0., 8.3});

    ClearanceReport rep; // even naming the heads changes nothing
    rep.routes.push_back(record(0, -1));
    rep.routes.push_back(record(1, -1));
    const HeadBridgeMetrics m = measure(b, rep);
    REQUIRE(m.aux_pillars == 1);
    REQUIRE(m.prop_bars == 1);
    REQUIRE(m.A_prop == 1);
    REQUIRE(m.revC_aux == 1);
    REQUIRE(m.revC_head + m.C_pillar + m.A == 0);
    REQUIRE(m.recorded == 0);
    REQUIRE(m.unrecorded == 2);
}

TEST_CASE("a bar across bracing is B unless it meets it within R", "[head_bridge_metrics]")
{
    // Bracing between T0 and T1, rising 0.4 mm per mm along x: z = 10 + 0.4 x.
    SupportTreeBuilder b;
    const long t0 = pillar(b, 0., 0., 20.), t1 = pillar(b, 10., 0., 20.);
    b.add_crossbridge_between(t0, t1, Vec3d{0., 0., 10.}, Vec3d{10., 0., 14.}, 0.4, false);

    SECTION("across, nothing shared") {
        const long p = pillar(b, 5., 3., 12.);
        head(b, 0, Vec3d{5., -3., 12.});
        b.add_bridge_to_pillar(0, Vec3d{5., 3., 12.}, p);
        const HeadBridgeMetrics m = measure(b);
        REQUIRE(m.B == 1);
        REQUIRE(m.B_shared == 0);
        REQUIRE(m.unrecorded == 1); // the bracing's, never recorded
    }
    SECTION("onto an end, meeting it within R of its axis") {
        head(b, 0, Vec3d{0., -3., 10.5});
        b.add_bridge_to_pillar(0, Vec3d{0., 0., 10.5}, t0);
        const HeadBridgeMetrics m = measure(b);
        REQUIRE(m.B == 0);
    }
    SECTION("onto a pillar hugging an end, within R of that pillar") {
        // q is 0.9 mm from t0, short of 0.5 + 0.5 + 0.2: one cluster. The
        // bar meets the bracing 0.24 mm from q's axis.
        const long q = pillar(b, 0.9, 0., 20.);
        head(b, 0, Vec3d{0.9, -3., 10.6});
        b.add_bridge_to_pillar(0, Vec3d{0.9, 0., 10.6}, q);
        const HeadBridgeMetrics m = measure(b);
        REQUIRE(m.B == 0);
    }
    SECTION("onto a pillar hugging an end, along the bracing outside R") {
        const long q = pillar(b, 0.9, 0., 20.);
        head(b, 0, Vec3d{6., 0., 12.1}); // 0.3 mm under the bracing
        b.add_bridge_to_pillar(0, Vec3d{0.9, 0., 10.06}, q);
        const HeadBridgeMetrics m = measure(b);
        REQUIRE(m.B == 1);
        REQUIRE(m.B_shared == 1);
    }
}

TEST_CASE("D, a thick head on a thin pillar, and bracing laid twice",
          "[head_bridge_metrics]")
{
    SupportTreeBuilder b;

    SECTION("a head with two bars down") {
        const long p0 = pillar(b, 3., 0., 18.3), p1 = pillar(b, -3., 0., 18.3);
        head(b, 0, Vec3d{0., 0., 18.3});
        b.add_bridge_to_pillar(0, Vec3d{3., 0., 18.3}, p0);
        b.add_bridge_to_pillar(0, Vec3d{-3., 0., 18.3}, p1);
        const HeadBridgeMetrics m = measure(b);
        REQUIRE(m.D == 1);
        REQUIRE(m.thick_on_thin == 0);
    }
    SECTION("a thick head onto a thin pillar, and an even one") {
        const long thin = pillar(b, 3., 0., 18.3, 0.5);
        const long even = pillar(b, 3., 10., 18.3, 0.75);
        head(b, 0, Vec3d{0., 0., 18.3}, 0.75);
        head(b, 1, Vec3d{0., 10., 18.3}, 0.75);
        b.add_bridge_to_pillar(0, Vec3d{3., 0., 18.3}, thin);
        b.add_bridge_to_pillar(1, Vec3d{3., 10., 18.3}, even);
        const HeadBridgeMetrics m = measure(b);
        REQUIRE(m.thick_on_thin == 1);
        REQUIRE(m.D == 0);
    }
    SECTION("one bar between a pair laid twice, and pairs beyond the pillars reached") {
        const long t0 = pillar(b, 0., 0., 20.), t1 = pillar(b, 10., 0., 20.);
        const long t2 = pillar(b, 0., 10., 20.), t3 = pillar(b, 10., 10., 20.);
        b.add_crossbridge_between(t0, t1, Vec3d{0., 0., 10.}, Vec3d{10., 0., 14.}, 0.4, false);
        b.add_crossbridge_between(t1, t0, Vec3d{10., 0., 14.}, Vec3d{0., 0., 10.}, 0.4, false);
        b.add_crossbridge_between(t2, t3, Vec3d{0., 10., 10.}, Vec3d{10., 10., 14.}, 0.4, false);
        b.add_crossbridge_between(t2, t3, Vec3d{10., 10., 10.}, Vec3d{0., 10., 14.}, 0.4, false);
        b.pillar(t0).pairs = 2; // one more than it reaches
        b.pillar(t1).pairs = 1;
        b.pillar(t2).pairs = 1;
        b.pillar(t3).pairs = 1;
        const HeadBridgeMetrics m = measure(b);
        REQUIRE(m.truss_bars == 4);
        REQUIRE(m.dup_truss_pairs == 1); // t0-t1; t2-t3 is a zig-zag
        REQUIRE(m.pairs_excess == 1);
    }
}

TEST_CASE("the report's counts are carried over", "[head_bridge_metrics]")
{
    SupportTreeBuilder b;
    ClearanceReport    rep;
    rep.l1     = 2;
    rep.l2     = 1;
    rep.none   = 3;
    rep.native = 4;
    rep.centroid.resize(5);

    const HeadBridgeMetrics m = measure(b, rep);
    REQUIRE(m.l1 == 2);
    REQUIRE(m.l2 == 1);
    REQUIRE(m.none == 3);
    REQUIRE(m.centroid_native == 4);
    REQUIRE(m.centroid_recorded == 5);

    const nlohmann::json j = nlohmann::json::parse(m.to_json());
    for (const char *k : {"A", "A_prop", "B", "B_shared", "C_pillar", "C_head", "revC_aux",
                          "revC_head", "revC_head_native", "revC_head_net", "recorded",
                          "unrecorded", "D", "thick_on_thin", "dup_truss_pairs", "pairs_excess",
                          "l1", "l2", "none", "centroid_recorded", "centroid_native"})
        REQUIRE(j.contains(k));
    REQUIRE(j.at("l2") == 1);
    REQUIRE(j.at("unrecorded_list").is_array());
}

// ─── code review of task 6.10 ────────────────────────────────────────────────

TEST_CASE("D counts bars leaving a head's own short pillar", "[head_bridge_metrics]")
{
    // The way DefaultSupportTree lays a bar to a pillar: a short pillar down
    // from the head, the bar from its bottom (not from the junction).
    SupportTreeBuilder b;
    const long p0 = pillar(b, 3., 0., 15.3), p1 = pillar(b, -3., 0., 15.3);
    head(b, 0, Vec3d{0., 0., 18.3});
    const long stem = b.add_pillar(long(0), 3.);
    REQUIRE(std::abs(b.pillar(stem).endpt.z() - 15.3) < 1e-9);

    SECTION("one bar: one way down") {
        b.add_bridge_between(p0, Vec3d{0., 0., 15.3}, Vec3d{3., 0., 15.3}, 0.5,
                             BridgeKind::head, stem);
        const HeadBridgeMetrics m = measure(b);
        REQUIRE(m.D == 0);
    }
    SECTION("two bars from the short pillar: D") {
        b.add_bridge_between(p0, Vec3d{0., 0., 15.3}, Vec3d{3., 0., 15.3}, 0.5,
                             BridgeKind::head, stem);
        b.add_bridge_between(p1, Vec3d{0., 0., 15.3}, Vec3d{-3., 0., 15.3}, 0.5,
                             BridgeKind::head, stem);
        const HeadBridgeMetrics m = measure(b);
        REQUIRE(m.D == 1);
    }
}

TEST_CASE("a record about an element no head owns names no other one",
          "[head_bridge_metrics]")
{
    // Head 1's bar runs 0.5 mm from pillar q, which no head's path owns.
    SupportTreeBuilder b;
    const long p1 = pillar(b, 8., 0., 18.3);
    head(b, 1, Vec3d{2., 0., 18.3});
    b.add_bridge_to_pillar(1, Vec3d{8., 0., 18.3}, p1);
    const long q = pillar(b, 5., 0.5, 30.);

    auto record_with = [](clearance::PrimKind with, long with_pillar) {
        ClearanceReport rep;
        RouteRecord     rec = record(1, -1);
        rec.contacts[0].with        = with;
        rec.contacts[0].with_pillar = with_pillar;
        rep.routes.push_back(rec);
        return rep;
    };

    SECTION("a record about something else of no head") {
        const HeadBridgeMetrics m = measure(b, record_with(clearance::PrimKind::prop_bar, -1));
        REQUIRE(m.C_pillar == 1);
        REQUIRE(m.recorded == 0);
        REQUIRE(m.unrecorded == 1);
    }
    SECTION("a record about another pillar of no head") {
        const HeadBridgeMetrics m = measure(b, record_with(clearance::PrimKind::pillar, p1));
        REQUIRE(m.recorded == 0);
        REQUIRE(m.unrecorded == 1);
    }
    SECTION("a record about this very pillar") {
        const HeadBridgeMetrics m = measure(b, record_with(clearance::PrimKind::pillar, q));
        REQUIRE(m.recorded == 1);
        REQUIRE(m.unrecorded == 0);
    }
}

TEST_CASE("support point spacing goes by the head that grew the pillar",
          "[head_bridge_metrics]")
{
    // Head 1 sits on top of pillar p, which it did not grow (a pillar from
    // the ground, its pillar_id set). Head 2 is 0.9 mm from head 1, natively
    // too close, and the same from p's axis. p touching head 2 is a reverse C
    // of its own, not support point spacing: nothing grew p from head 1.
    SupportTreeBuilder b;
    const long p = pillar(b, 0., 0., 18.3);
    head(b, 1, Vec3d{0., 0., 18.3});
    b.head(1).pillar_id = p;
    head(b, 2, Vec3d{0.9, 0., 18.3});

    const HeadBridgeMetrics m = measure(b);
    REQUIRE(m.revC_head == 1);
    REQUIRE(m.revC_head_native == 0);
    REQUIRE(m.revC_head_net == 1);
    REQUIRE(m.unrecorded == 1);
}

TEST_CASE("a route chains on from route bars only", "[head_bridge_metrics]")
{
    // A route bar starting where head 0's bar ended, on pillar p's top, is
    // not head 0's: nothing ties it to a head, and it stays unassigned.
    SupportTreeBuilder b;
    const long p = pillar(b, 3., 0., 18.3);
    head(b, 0, Vec3d{0., 0., 18.3});
    b.add_bridge_to_pillar(0, Vec3d{3., 0., 18.3}, p);
    b.add_bridge(Vec3d{3., 0., 18.3}, Vec3d{6., 0., 15.}, 0.5, BridgeKind::route);

    const HeadBridgeMetrics m = head_bridge_metrics(b, {});
    REQUIRE(m.route_bars == 1);
    REQUIRE(m.unassigned == 1);
}

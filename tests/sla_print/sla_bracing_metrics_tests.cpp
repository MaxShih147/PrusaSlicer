// Bracing measurement (openspec change sla-bracing-planar-topology, task group 1).
//
// The first block pins what bracing_metrics() counts, on hand-built builders, so
// a number in the baseline report means exactly what the spec's acceptance rule
// means. The last test case is hidden ("[.bracing_baseline]"): it runs reference
// models through the real pipeline and is only started on request, with a plan
// file named by BRACING_BASELINE_PLAN. So is "[.bracing_io]", which exports the
// same models in both exchange formats and checks kind and pairs (task 5.3).
// "[.head_bridge_acceptance]" (openspec change sla-head-bridge-clearance, task
// 6.6) measures the head routes of the same kind of plan with
// head_bridge_metrics(), each case on a fixed support point list.

#include <catch2/catch_test_macros.hpp>
#include <test_utils.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>

#include <nlohmann/json.hpp>
#include <oneapi/tbb/parallel_for.h>
#include <oneapi/tbb/task_arena.h>

#include "libslic3r/FileReader.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/SLAPrint.hpp"
#include "libslic3r/SLA/DefaultSupportTree.hpp"
#include "libslic3r/SLA/PriorPillarIO.hpp"
#include "libslic3r/SLA/SupportPointIO.hpp"
#include "libslic3r/SLA/SupportTree.hpp"
#include "libslic3r/SLA/SupportTreeIO.hpp"

#include "sla_bracing_metrics.hpp"
#include "sla_head_bridge_metrics.hpp"

using namespace Slic3r;
using namespace Slic3r::sla;

namespace {

long pillar_at(SupportTreeBuilder &b, double x, double y, double h = 20., double r = 0.5)
{
    return b.add_pillar(Vec3d{x, y, 0.}, h, r);
}

// One bar per group is enough: the plan rules look at the pillar axes.
void truss(SupportTreeBuilder &b, long p, long q, double r = 0.5, bool relaxed = false)
{
    const Pillar &a = b.pillar(p), &c = b.pillar(q);
    b.add_crossbridge_between(p, q, a.startpoint() - Vec3d{0, 0, 5},
                              c.startpoint() - Vec3d{0, 0, 2}, r, relaxed);
}

Vec2d polar(double deg, double len)
{
    return {len * std::cos(deg * PI / 180.), len * std::sin(deg * PI / 180.)};
}

} // namespace

TEST_CASE("bracing_metrics counts an X crossing once", "[bracing_metrics]")
{
    SupportTreeBuilder b;
    const long p0 = pillar_at(b, 0, 0), p1 = pillar_at(b, 10, 0);
    const long p2 = pillar_at(b, 5, -5), p3 = pillar_at(b, 5, 5);
    truss(b, p0, p1);
    truss(b, p2, p3);

    BracingMetrics m = bracing_metrics(b, 0.);
    REQUIRE(m.truss_groups == 2);
    REQUIRE(m.cross_truss == 1);
    REQUIRE(m.shared_angle == 0);
    REQUIRE(m.groups_in_plan_violation == 2);
}

TEST_CASE("bracing_metrics plan gap is judged strictly below r1 + r2 + c", "[bracing_metrics]")
{
    const double limit = 0.5 + 0.5 + BracingMetrics::SAFETY_CLEARANCE_MM;

    // Pillars thinner than the bars, so they do not hug at this gap.
    auto run = [](double gap) {
        SupportTreeBuilder b;
        truss(b, pillar_at(b, 0, 0, 20., 0.3), pillar_at(b, 10, 0, 20., 0.3));
        truss(b, pillar_at(b, 0, gap, 20., 0.3), pillar_at(b, 10, gap, 20., 0.3));
        return bracing_metrics(b, 0.).cross_truss;
    };

    REQUIRE(run(limit + 1e-6) == 0);
    REQUIRE(run(limit - 1e-6) == 1);
}

TEST_CASE("bracing_metrics treats hugging pillars as one", "[bracing_metrics]")
{
    // Two parallel groups whose end pillars hug: they share the clusters,
    // so it is the angle (0 deg) that is broken, not the gap.
    SupportTreeBuilder b;
    truss(b, pillar_at(b, 0, 0), pillar_at(b, 10, 0));
    truss(b, pillar_at(b, 0, 0.9), pillar_at(b, 10, 0.9));
    BracingMetrics m = bracing_metrics(b, 0.);
    REQUIRE(m.clusters == 2);
    REQUIRE(m.max_cluster == 2);
    REQUIRE(m.cross_truss == 0);
    REQUIRE(m.shared_angle == 1);
    // The group passes 0.9 from the other end's twin: not a third pillar.
    REQUIRE(m.obstacle_pillar == 0);

    // A group leaving the cluster at 45 deg is fine.
    SupportTreeBuilder c;
    const long a = pillar_at(c, 0, 0), a2 = pillar_at(c, 0.9, 0);
    truss(c, a, pillar_at(c, 10, 0));
    truss(c, a2, pillar_at(c, 0.9 + 6., 6.));
    m = bracing_metrics(c, 0.);
    REQUIRE(m.shared_angle == 0);
    REQUIRE(m.cross_truss == 0);
    REQUIRE(m.obstacle_pillar == 0);
}

TEST_CASE("bracing_metrics counts relaxed groups apart", "[bracing_metrics]")
{
    SupportTreeBuilder b;
    truss(b, pillar_at(b, 0, 0), pillar_at(b, 6, 6));
    truss(b, pillar_at(b, 0, 6), pillar_at(b, 6, 0), 0.5, true); // a rescue
    const BracingMetrics m = bracing_metrics(b, 0.);
    REQUIRE(m.relaxed_groups == 1);
    REQUIRE(m.cross_truss == 0);
    REQUIRE(m.cross_truss_relaxed == 1);
    REQUIRE(m.groups_in_plan_violation == 0);
}

TEST_CASE("bracing_metrics shared pillar angle limit is 20 degrees", "[bracing_metrics]")
{
    auto run = [](double deg) {
        SupportTreeBuilder b;
        const long s = pillar_at(b, 0, 0);
        const Vec2d u = polar(0., 8.), v = polar(deg, 8.);
        truss(b, s, pillar_at(b, u.x(), u.y()));
        truss(b, s, pillar_at(b, v.x(), v.y()));
        return bracing_metrics(b, 0.);
    };

    BracingMetrics narrow = run(19.);
    REQUIRE(narrow.shared_angle == 1);
    REQUIRE(narrow.cross_truss == 0);

    BracingMetrics wide = run(21.);
    REQUIRE(wide.shared_angle == 0);
}

TEST_CASE("bracing_metrics counts a truss passing a third pillar axis", "[bracing_metrics]")
{
    const double limit = 0.5 + 0.5 + BracingMetrics::SAFETY_CLEARANCE_MM;

    auto run = [](double offset) {
        SupportTreeBuilder b;
        truss(b, pillar_at(b, 0, 0), pillar_at(b, 10, 0));
        pillar_at(b, 5, offset);
        return bracing_metrics(b, 0.);
    };

    BracingMetrics through = run(limit - 1e-6);
    REQUIRE(through.obstacle_pillar == 1);
    REQUIRE(through.groups_in_obstacle_violation == 1);
    REQUIRE(through.obstacle_pillar_offground == 0);

    REQUIRE(run(limit + 1e-6).obstacle_pillar == 0);

    // A pillar standing on the model still blocks in plan view; it is counted
    // in the off-ground share as well.
    SupportTreeBuilder b;
    truss(b, pillar_at(b, 0, 0), pillar_at(b, 10, 0));
    b.add_pillar(Vec3d{5, 0, 3.}, 10., 0.5);
    BracingMetrics up = bracing_metrics(b, 0.);
    REQUIRE(up.obstacle_pillar == 1);
    REQUIRE(up.obstacle_pillar_offground == 1);
}

TEST_CASE("bracing_metrics isolated tall pillars and partners", "[bracing_metrics]")
{
    SupportTreeBuilder b;
    pillar_at(b, 0, 0, 20.);                 // isolated, tall
    pillar_at(b, 50, 0, 15.);                // not taller than 15 mm
    const long linked = pillar_at(b, 100, 0, 20.);
    b.increment_links(b.pillar(linked));
    const long up = b.add_pillar(Vec3d{150, 0, 3.}, 20., 0.5); // stands on the model

    BracingMetrics m = bracing_metrics(b, 0.);
    REQUIRE(m.tall == 3);
    REQUIRE(m.isolated_tall == 2);
    REQUIRE(m.isolated_tall_ground == 1);
    (void) up;

    // A hub with four partners.
    SupportTreeBuilder h;
    const long hub = pillar_at(h, 0, 0);
    for (double deg : {0., 90., 180., 270.}) {
        const Vec2d p = polar(deg, 8.);
        truss(h, hub, pillar_at(h, p.x(), p.y()));
    }
    BracingMetrics hm = bracing_metrics(h, 0.);
    REQUIRE(hm.max_partners == 4);
    REQUIRE(hm.partners_histogram[4] == 1);
    REQUIRE(hm.partners_histogram[1] == 4);
    REQUIRE(hm.shared_angle == 0);
}

TEST_CASE("bracing_metrics tells bars apart by where the builder keeps them", "[bracing_metrics]")
{
    SupportTreeBuilder b;
    const long main = pillar_at(b, 0, 0);
    const long other = pillar_at(b, 10, 0);
    const long prop = pillar_at(b, 3, 3);
    b.pillar(prop).props_for = main;

    truss(b, main, other);
    b.add_bridge_between(other, {5, 5, 25}, {10, 0, 20}, 0.4, BridgeKind::head);
    b.add_bridge_between(prop, {0, 0, 18}, {3, 3, 18}, 0.4, BridgeKind::prop, main);
    b.add_bridge({20, 0, 10}, {22, 0, 8}, 0.4, BridgeKind::route);
    b.add_diffbridge(Vec3d{30, 0, 10}, Vec3d{31, 0, 8}, 0.4, 0.6);   // route, widening
    b.add_crossbridge(Vec3d{40, 0, 10}, Vec3d{41, 0, 8}, 0.4);       // truss, no owners

    BracingMetrics m = bracing_metrics(b, 0.);
    REQUIRE(m.aux_pillars == 1);
    REQUIRE(m.truss_bars == 2);
    REQUIRE(m.orphan_truss == 1);
    REQUIRE(m.truss_groups == 1);
    REQUIRE(m.head_bars == 1);
    REQUIRE(m.prop_bars == 1);
    REQUIRE(m.route_bars == 1);
    REQUIRE(m.route_diff == 1);
    REQUIRE(m.kind_unset == 0);
    REQUIRE(m.kind_disagrees == 0);
}

// Task 2.2: the kind is written by the builder function that makes the bar.
TEST_CASE("builder functions tag every bar with its kind", "[bracing_metrics]")
{
    SupportTreeBuilder b;
    const long p = pillar_at(b, 0, 0), q = pillar_at(b, 10, 0);
    b.add_head(0, 1., 0.2, 1., 0.2, DOWN, Vec3d{0, 0, 30});

    b.add_crossbridge_between(p, q, {0, 0, 10}, {10, 0, 15}, 0.5, false);
    b.add_crossbridge(Vec3d{0, 0, 5}, Vec3d{10, 0, 8}, 0.5);
    b.add_bridge_to_pillar(0, {0, 0, 20}, p);
    b.add_bridge(0, Vec3d{0, 0, 22});
    b.add_bridge_between(q, {5, 0, 20}, {10, 0, 20}, 0.4, BridgeKind::head);
    b.add_bridge_between(q, {0, 0, 18}, {10, 0, 18}, 0.4, BridgeKind::prop, p);
    b.add_bridge({20, 0, 10}, {22, 0, 8}, 0.4, BridgeKind::route);
    b.add_diffbridge(Vec3d{30, 0, 10}, Vec3d{31, 0, 8}, 0.4, 0.6);
    b.add_diffbridge(DiffBridge{Vec3d{40, 0, 10}, Vec3d{41, 0, 8}, 0.4, 0.6});

    for (const Bridge &br : b.crossbridges()) REQUIRE(br.kind == BridgeKind::truss);
    for (const DiffBridge &br : b.diffbridges()) REQUIRE(br.kind == BridgeKind::route);

    const auto &bars = b.bridges();
    REQUIRE(bars.size() == 5);
    REQUIRE(bars[0].kind == BridgeKind::head);
    REQUIRE(bars[1].kind == BridgeKind::head);
    REQUIRE(bars[2].kind == BridgeKind::head);
    REQUIRE(bars[3].kind == BridgeKind::prop);
    REQUIRE(bars[4].kind == BridgeKind::route);
}

TEST_CASE("builder functions write the owners with the bar", "[bracing_metrics]")
{
    SupportTreeBuilder b;
    const long p = pillar_at(b, 0, 0), q = pillar_at(b, 10, 0);
    b.add_head(0, 1., 0.2, 1., 0.2, DOWN, Vec3d{0, 0, 30});

    const Bridge t = b.add_crossbridge_between(p, q, {0, 0, 10}, {10, 0, 15}, 0.5, true, true);
    REQUIRE((t.owner_a == p && t.owner_b == q && t.relaxed && t.fallback));

    const Bridge h = b.add_bridge_to_pillar(0, {0, 0, 20}, p);
    const Bridge g = b.add_bridge({20, 0, 10}, {22, 0, 8}, 0.4, BridgeKind::route, q);
    const Bridge u = b.add_bridge({30, 0, 10}, {32, 0, 8}, 0.4, BridgeKind::route);
    const DiffBridge d = b.add_diffbridge(Vec3d{40, 0, 10}, Vec3d{41, 0, 8}, 0.4, 0.6);

    // What the caller got is what the builder keeps.
    REQUIRE(b.bridges()[size_t(h.id)].owner_b == p);
    REQUIRE(b.bridges()[size_t(g.id)].owner_b == q);
    REQUIRE(b.bridges()[size_t(g.id)].owner_a == SupportTreeNode::ID_UNSET);
    REQUIRE(b.bridges()[size_t(u.id)].owner_b == SupportTreeNode::ID_UNSET);

    // A bar laid before its pillar is given the pillar afterwards.
    b.own_bridge(u.id, p);
    b.own_diffbridge(d.id, q);
    REQUIRE(b.bridges()[size_t(u.id)].owner_b == p);
    REQUIRE(b.diffbridges()[size_t(d.id)].owner_b == q);
}

TEST_CASE("bars added from many threads keep their own owners", "[bracing_metrics]")
{
    // Routing adds bars in parallel. Every bar carries its index in its
    // geometry and in its owners; a bar whose owners were written after the
    // lock (into a vector another thread was growing) would not match.
    SupportTreeBuilder b;
    const int n = 20000;
    std::vector<long> got(size_t(n), -1);
    tbb::parallel_for(0, n, [&](int i) {
        const Vec3d s{double(i), 0., 10.}, e{double(i), 1., 9.};
        switch (i % 3) {
        case 0: got[size_t(i)] = b.add_bridge_between(i, s, e, 0.4, BridgeKind::head, i + n).id; break;
        case 1: got[size_t(i)] = b.add_bridge(s, e, 0.4, BridgeKind::route, i).id; break;
        default: got[size_t(i)] = b.add_crossbridge_between(i + n, i, s, e, 0.4, false).id; break;
        }
    });

    REQUIRE(b.bridges().size() + b.crossbridges().size() == size_t(n));
    for (int i = 0; i < n; ++i) {
        const Bridge &br = i % 3 == 2 ? b.crossbridges()[size_t(got[size_t(i)])]
                                      : b.bridges()[size_t(got[size_t(i)])];
        CAPTURE(i);
        REQUIRE(br.startp.x() == double(i)); // the id handed back is this bar
        REQUIRE(br.owner_b == i);
        REQUIRE(br.owner_a == (i % 3 == 1 ? SupportTreeNode::ID_UNSET : long(i + n)));
    }
}

TEST_CASE("pair and link counts are kept apart", "[bracing_metrics]")
{
    SupportTreeBuilder b;
    const long p = pillar_at(b, 0, 0);
    b.increment_pairs(b.pillar(p));
    b.increment_pairs(b.pillar(p));
    b.increment_links(b.pillar(p));
    REQUIRE(b.pillar(p).pairs == 2);
    REQUIRE(b.pillar(p).links == 1);

    const long f = b.add_frozen_pillar({5, 0, 0}, 20., 0.5, 0.5, 1, 0, 3);
    REQUIRE(b.pillar(f).frozen);
    REQUIRE(b.pillar(f).links == 1);
    REQUIRE(b.pillar(f).pairs == 3);
}

TEST_CASE("builder_digests changes with any bar", "[bracing_metrics]")
{
    SupportTreeBuilder a, b;
    truss(a, pillar_at(a, 0, 0), pillar_at(a, 10, 0), 0.5);
    truss(b, pillar_at(b, 0, 0), pillar_at(b, 10, 0), 0.5000001);

    BuilderDigests da = builder_digests(a), db = builder_digests(b);
    REQUIRE(da.pillars == db.pillars);
    REQUIRE(da.crossbridges != db.crossbridges);
    REQUIRE(builder_digests(a).crossbridges == da.crossbridges);
}

// ─── Baseline runner ─────────────────────────────────────────────────────────
//
// Plan file (JSON):
//   { "out": "<path of the JSON Lines result file>",
//     "cases": [ { "id": "small", "model": "<stl>", "config": "<ini>",
//                  "repeats": [<runs with all threads>, <runs single threaded>],
//                  "point_runs": <how many times to regenerate the points> } ] }
//
// The support points are produced by SLAPrint up to slaposSupportPoints with the
// given config - the backend's own recipe - and the tree is then built straight
// from them, so the builder can be read afterwards. Building it here instead of
// through SLAPrint is what makes the builder reachable; the inputs are the ones
// SLAPrint::Steps::support_tree() assembles.

namespace {

using Clock = std::chrono::steady_clock;

double ms_since(Clock::time_point t0, Clock::time_point t1)
{
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

std::string points_digest(const SupportPoints &pts)
{
    uint64_t h = 1469598103934665603ull;
    auto mix = [&h](const void *p, size_t n) {
        auto *c = static_cast<const unsigned char *>(p);
        for (size_t i = 0; i < n; ++i) { h ^= c[i]; h *= 1099511628211ull; }
    };
    for (const SupportPoint &sp : pts) {
        mix(sp.pos.data(), sizeof(float) * 3);
        mix(&sp.head_front_radius, sizeof(sp.head_front_radius));
        mix(&sp.type, sizeof(sp.type));
    }
    std::ostringstream o;
    o << std::hex << std::setw(16) << std::setfill('0') << h;
    return o.str();
}

DynamicPrintConfig load_backend_config(const std::string &ini)
{
    // The CLI's own sequence (LoadPrintData.cpp): the --load file over the SLA
    // defaults, normalized, technology forced to SLA.
    DynamicPrintConfig loaded;
    loaded.load(ini, ForwardCompatibilitySubstitutionRule::Enable);
    loaded.normalize_fdm();

    DynamicPrintConfig cfg;
    cfg.apply(loaded);
    cfg.option<ConfigOptionEnum<PrinterTechnology>>("printer_technology", true)->value = ptSLA;

    SLAFullPrintConfig full;
    full.apply(cfg, true);
    cfg.apply(full, true);
    return cfg;
}

struct PreparedCase
{
    std::shared_ptr<const indexed_triangle_set> mesh;
    SupportPoints        pts;
    SupportTreeConfig    tree_cfg;
    PadConfig            pad_cfg;
    double               zoffset = 0.;
    double               points_ms = 0.;
};

// The tall fixture: a model built here so the bracing measurement also covers
// auxiliary pillars, which none of the plate fixtures grows. A central post with
// two rings of rising arms; each arm ends in a small pad whose underside takes
// a support. The pads sit far enough apart (> max_pillar_link_distance) that
// their pillars find no neighbour, and high enough that every one of them is a
// lonely pillar over max_solo_pillar_height (inner ring) or over
// max_dual_pillar_height (outer ring) - the two cases that grow props.
// The arms rise at 55 degrees, steeper than the 45 degree overhang limit, so
// they carry no supports of their own. Plan name: "builtin:tall_fixture".
TriangleMesh make_tall_fixture()
{
    TriangleMesh mesh = make_cube(4., 4., 30.);
    mesh.translate(-2.f, -2.f, 0.f);

    auto ring = [&mesh](int arms, double azimuth0_deg, double radius, double rise_deg) {
        const double rise = rise_deg * PI / 180.;
        const double z0   = 4.;
        const double len  = radius / std::cos(rise);
        for (int i = 0; i < arms; ++i) {
            const double az = (azimuth0_deg + 360. * i / arms) * PI / 180.;

            TriangleMesh arm = make_cube(len, 2., 2.);
            arm.translate(0.f, -1.f, -1.f);
            arm.rotate_y(float(-rise));
            arm.rotate_z(float(az));
            arm.translate(0.f, 0.f, float(z0));
            mesh.merge(arm);

            TriangleMesh pad = make_cube(6., 6., 1.5);
            pad.translate(-3.f, -3.f, -0.75f);
            pad.translate(float(radius * std::cos(az)), float(radius * std::sin(az)),
                          float(z0 + radius * std::tan(rise)));
            mesh.merge(pad);
        }
    };
    ring(6, 0., 16., 55.);  // pads near z 27: pillars between 15 and 35 mm
    ring(6, 30., 28., 55.); // pads near z 44: pillars over 35 mm

    return mesh;
}

std::string mesh_digest(const indexed_triangle_set &its)
{
    uint64_t h = 1469598103934665603ull;
    auto mix = [&h](const void *p, size_t n) {
        auto *c = static_cast<const unsigned char *>(p);
        for (size_t i = 0; i < n; ++i) { h ^= c[i]; h *= 1099511628211ull; }
    };
    for (const auto &v : its.vertices) mix(v.data(), sizeof(float) * 3);
    for (const auto &f : its.indices) mix(f.data(), sizeof(int) * 3);
    std::ostringstream o;
    o << std::hex << std::setw(16) << std::setfill('0') << h;
    return o.str();
}

Model load_case_model(const std::string &stl)
{
    if (stl == "builtin:tall_fixture") {
        Model model;
        ModelObject *o = model.add_object("tall_fixture", "", make_tall_fixture());
        o->add_instance();
        return model;
    }
    return FileReader::load_model(stl);
}

// `points`, when given: a support point list to use instead of generating one,
// loaded the way the CLI's --import-support-points loads it (fingerprint not
// checked).
PreparedCase prepare(const std::string &stl, const DynamicPrintConfig &cfg,
                     const std::string &points = {})
{
    Model model = load_case_model(stl);
    for (ModelObject *o : model.objects) o->ensure_on_bed();
    const double w = cfg.opt_float("display_width"), h = cfg.opt_float("display_height");
    model.center_instances_around_point({w / 2., h / 2.});

    if (!points.empty()) {
        std::ifstream in(points);
        REQUIRE(in.good());
        std::ostringstream text;
        text << in.rdbuf();

        SLAPrintObjectConfig obj_cfg;
        obj_cfg.apply(cfg, true);
        SupportPointFile file;
        std::string      err;
        REQUIRE(support_points_from_string(text.str(), make_support_cfg(obj_cfg), file, err));
        REQUIRE(model.objects.size() == 1);
        model.objects.front()->sla_support_points = file.points;
        model.objects.front()->sla_points_status  = PointsStatus::UserModified;
    }

    SLAPrint print;
    print.set_status_callback([](const PrintBase::SlicingStatus &) {});
    print.apply(model, cfg);
    PrintBase::TaskParams task;
    task.to_object_step = slaposSupportPoints;
    print.set_task(task);

    const auto t0 = Clock::now();
    print.process();
    const auto t1 = Clock::now();

    REQUIRE(print.objects().size() == 1);
    const SLAPrintObject &po = *print.objects().front();

    PreparedCase pc;
    pc.mesh      = po.get_mesh_to_print();
    pc.pts       = po.get_support_points();
    pc.tree_cfg  = make_support_cfg(po.config());
    pc.pad_cfg   = make_pad_cfg(po.config());
    // support_points() takes the bottom of the mesh to slice; with no hollowing
    // and a single part that is the bottom of this very mesh.
    pc.zoffset   = bounding_box(*pc.mesh).min.z();
    pc.points_ms = ms_since(t0, t1);

    if (is_zero_elevation(po.config())) {
        const float lvl = float(pc.zoffset + EPSILON);
        pc.pts.erase(std::remove_if(pc.pts.begin(), pc.pts.end(),
                                    [lvl](const SupportPoint &sp) { return sp.pos.z() <= lvl; }),
                     pc.pts.end());
    }
    return pc;
}

nlohmann::json run_tree(const SupportableMesh &sm)
{
    Clock::time_point t_inter{}, t_merge{}, t_done{};
    JobController ctl;
    ctl.statuscb = [&](unsigned, const std::string &msg) {
        const auto now = Clock::now();
        if (msg == "Interconnecting pillars") t_inter = now;
        else if (msg == "Merging support mesh") t_merge = now;
        else if (msg == "Done") t_done = now;
    };

    SupportTreeBuilder builder(ctl);
    const auto t0 = Clock::now();
    DefaultSupportTree::execute(builder, sm);
    const auto t1 = Clock::now();
    REQUIRE(t_inter != Clock::time_point{});
    REQUIRE(t_merge != Clock::time_point{});

    const BracingMetrics m = bracing_metrics(builder, ground_level(sm));
    const BuilderDigests d = builder_digests(builder);

    nlohmann::json j;
    j["tree_ms"]         = ms_since(t0, t1);
    j["interconnect_ms"] = ms_since(t_inter, t_merge);
    j["merge_ms"]        = ms_since(t_merge, t_done == Clock::time_point{} ? t1 : t_done);
    j["metrics"]         = nlohmann::json::parse(m.to_json());
    j["digest"] = {{"heads", d.heads},           {"pillars", d.pillars},
                   {"bridges", d.bridges},       {"crossbridges", d.crossbridges},
                   {"diffbridges", d.diffbridges}, {"mesh", d.mesh},
                   {"mesh_vertices", d.mesh_vertices}, {"mesh_faces", d.mesh_faces}};
    return j;
}

} // namespace

TEST_CASE("bracing baseline over reference models", "[.bracing_baseline]")
{
    const char *plan_path = std::getenv("BRACING_BASELINE_PLAN");
    REQUIRE(plan_path != nullptr);

    std::ifstream plan_in(plan_path);
    REQUIRE(plan_in.good());
    const nlohmann::json plan = nlohmann::json::parse(plan_in);

    std::ofstream out(plan.at("out").get<std::string>(), std::ios::app);
    REQUIRE(out.good());

    auto emit = [&out](nlohmann::json j) {
        out << j.dump() << '\n';
        out.flush();
        std::cout << j.dump() << std::endl;
    };

    const int hw_threads = tbb::this_task_arena::max_concurrency();

    for (const auto &c : plan.at("cases")) {
        const std::string id = c.at("id").get<std::string>();
        const DynamicPrintConfig cfg = load_backend_config(c.at("config").get<std::string>());

        PreparedCase pc;
        const int point_runs = c.value("point_runs", 1);
        for (int k = 0; k < point_runs; ++k) {
            PreparedCase again = prepare(c.at("model").get<std::string>(), cfg);
            emit({{"case", id}, {"kind", "points"}, {"run", k},
                  {"points", again.pts.size()}, {"points_digest", points_digest(again.pts)},
                  {"points_ms", again.points_ms}, {"threads", hw_threads},
                  {"input_mesh_digest", mesh_digest(*again.mesh)},
                  {"input_mesh_faces", again.mesh->indices.size()}});
            if (k == 0) pc = std::move(again);
        }

        SupportableMesh sm{*pc.mesh, pc.pts, pc.tree_cfg};
        sm.pad_cfg = pc.pad_cfg;
        sm.zoffset = pc.zoffset;

        const auto &repeats = c.at("repeats");
        const int   runs_mt = repeats.at(0).get<int>(), runs_st = repeats.at(1).get<int>();

        for (int k = 0; k < runs_mt; ++k) {
            nlohmann::json j = run_tree(sm);
            j["case"] = id; j["kind"] = "tree"; j["run"] = k; j["threads"] = hw_threads;
            j["ground_z"] = ground_level(sm);
            emit(j);
        }

        tbb::task_arena single(1);
        for (int k = 0; k < runs_st; ++k) {
            nlohmann::json j;
            single.execute([&] { j = run_tree(sm); });
            j["case"] = id; j["kind"] = "tree"; j["run"] = k; j["threads"] = 1;
            j["ground_z"] = ground_level(sm);
            emit(j);
        }
    }
}

TEST_CASE("reference models export a kind for every bar", "[.bracing_io]")
{
    const char *plan_path = std::getenv("BRACING_BASELINE_PLAN");
    REQUIRE(plan_path != nullptr);

    std::ifstream plan_in(plan_path);
    REQUIRE(plan_in.good());
    const nlohmann::json plan = nlohmann::json::parse(plan_in);

    static const std::set<std::string> known = {"truss", "head", "prop", "route"};

    for (const auto &c : plan.at("cases")) {
        const std::string id = c.at("id").get<std::string>();
        CAPTURE(id);
        const DynamicPrintConfig cfg = load_backend_config(c.at("config").get<std::string>());
        const PreparedCase pc = prepare(c.at("model").get<std::string>(), cfg);

        SupportableMesh sm{*pc.mesh, pc.pts, pc.tree_cfg};
        sm.pad_cfg = pc.pad_cfg;
        sm.zoffset = pc.zoffset;

        // What the CLI's --export-support-tree and --export-support-pillars
        // write, from the call SLAPrint makes.
        JobController       ctl;
        PriorPillars        grown;
        PriorAttachments    attached;
        SupportTreeElements els;
        create_support_tree(sm, ctl, &grown, &attached, nullptr, &els);

        const nlohmann::json tree =
            nlohmann::json::parse(support_tree_to_string(els, ground_level(sm)));
        REQUIRE(tree.at("version") == 2);
        REQUIRE_FALSE(tree.at("bridges").empty());

        std::map<std::string, size_t> kinds;
        for (const nlohmann::json &b : tree.at("bridges")) {
            REQUIRE(b.contains("kind"));
            REQUIRE(b.at("kind").is_string());
            const std::string k = b.at("kind").get<std::string>();
            REQUIRE(known.count(k) == 1);
            ++kinds[k];
        }
        for (const nlohmann::json &p : tree.at("pillars")) {
            REQUIRE(p.contains("pairs"));
            REQUIRE(p.at("pairs").get<unsigned>() >= p.at("links").get<unsigned>());
        }

        const nlohmann::json list =
            nlohmann::json::parse(prior_pillars_to_string(grown, ground_level(sm), attached));
        REQUIRE(list.at("version") == 1);
        REQUIRE(list.at("pillars").size() == grown.size());
        for (size_t i = 0; i < grown.size(); ++i) {
            REQUIRE(list.at("pillars").at(i).contains("pairs"));
            REQUIRE(list.at("pillars").at(i).at("pairs") == grown[i].pairs);
        }

        nlohmann::json j = {{"case", id}, {"bridges", tree.at("bridges").size()},
                            {"pillars", tree.at("pillars").size()}, {"kinds", kinds}};
        std::cout << j.dump() << std::endl;
    }
}

TEST_CASE("head route metrics over reference models", "[.head_bridge_acceptance]")
{
    // Plan: {"out": FILE, "cases": [{"id", "model", "config", "points",
    // "repeats": [threaded runs, single thread runs]}]}. "points" is a fixed
    // support point list (evidence/baseline-*/inputs of the change); without
    // it the points are generated. One JSON line per run.
    const char *plan_path = std::getenv("HEAD_BRIDGE_PLAN");
    REQUIRE(plan_path != nullptr);

    std::ifstream plan_in(plan_path);
    REQUIRE(plan_in.good());
    const nlohmann::json plan = nlohmann::json::parse(plan_in);

    std::ofstream out(plan.at("out").get<std::string>(), std::ios::app);
    REQUIRE(out.good());

    const int hw_threads = tbb::this_task_arena::max_concurrency();

    for (const auto &c : plan.at("cases")) {
        const std::string id = c.at("id").get<std::string>();
        CAPTURE(id);
        const DynamicPrintConfig cfg = load_backend_config(c.at("config").get<std::string>());
        const PreparedCase pc = prepare(c.at("model").get<std::string>(), cfg,
                                        c.value("points", std::string{}));

        SupportableMesh sm{*pc.mesh, pc.pts, pc.tree_cfg};
        sm.pad_cfg = pc.pad_cfg;
        sm.zoffset = pc.zoffset;

        auto run = [&](int k, int threads) {
            SupportTreeBuilder builder;
            ClearanceReport    report;
            DefaultSupportTree::execute(builder, sm, nullptr, &report);

            const HeadBridgeMetrics m = head_bridge_metrics(builder, report);
            const BuilderDigests    d = builder_digests(builder);
            nlohmann::json j = {{"case", id}, {"run", k}, {"threads", threads},
                                {"points", sm.pts.size()},
                                {"metrics", nlohmann::json::parse(m.to_json())},
                                {"digest", {{"heads", d.heads}, {"pillars", d.pillars},
                                            {"bridges", d.bridges},
                                            {"crossbridges", d.crossbridges},
                                            {"diffbridges", d.diffbridges}}}};
            out << j.dump() << '\n';
            out.flush();
            std::cout << j.dump() << std::endl;
        };

        const nlohmann::json repeats =
            c.contains("repeats") ? c.at("repeats") : nlohmann::json::array({1, 0});
        for (int k = 0; k < repeats.at(0).get<int>(); ++k)
            run(k, hw_threads);

        tbb::task_arena single(1);
        for (int k = 0; k < repeats.at(1).get<int>(); ++k)
            single.execute([&] { run(k, 1); });
    }
}

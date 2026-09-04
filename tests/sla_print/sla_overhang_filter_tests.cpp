///|/ Tests for the shared overhang angle predicate.
///|/
///|/ openspec change align-support-point-overhang-filter,
///|/ capability sla-overhang-threshold-semantics.
///|/
#include <catch2/catch_test_macros.hpp>
#include <test_utils.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <utility>

#include "libslic3r/FileReader.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/MeshNormals.hpp"
#include "libslic3r/SLAPrint.hpp"
#include "libslic3r/SLA/ModelFingerprint.hpp"
#include "libslic3r/SLA/SupportPointIO.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/SLA/DefaultSupportTree.hpp"
#include "libslic3r/SLA/SupportTree.hpp"
#include "libslic3r/SLA/SupportTreeBuilder.hpp"

using namespace Slic3r;
using namespace Slic3r::sla;

namespace {

constexpr double PI_ = M_PI;

double deg2rad(double d) { return d * PI_ / 180.; }

// The polar angle of the normal of a surface whose slope from the horizontal
// plane is `slope_deg`. 0 deg of slope is a flat down-facing face (normal
// straight down, polar = 180 deg); 90 deg is a vertical wall (polar = 90 deg).
double polar_for_slope(double slope_deg) { return PI_ - deg2rad(slope_deg); }

} // namespace

// The spec's boundary scenarios, spelled out one by one.
//
// The predicate passes a surface when its slope from the horizontal plane is
// at most (90 deg - support_critical_angle), so a SMALLER critical angle
// supports MORE surfaces.

TEST_CASE("Flat down-facing surfaces pass at every threshold", "[SLAOverhangFilter]")
{
    const double polar = polar_for_slope(0.); // normal straight down

    REQUIRE(passes_overhang_filter(polar, deg2rad(0.)));
    REQUIRE(passes_overhang_filter(polar, deg2rad(45.)));
    REQUIRE(passes_overhang_filter(polar, deg2rad(90.)));
}

TEST_CASE("Vertical walls pass only at a zero threshold", "[SLAOverhangFilter]")
{
    const double polar = polar_for_slope(90.); // vertical wall

    REQUIRE(passes_overhang_filter(polar, deg2rad(0.)));
    REQUIRE_FALSE(passes_overhang_filter(polar, deg2rad(45.)));
    REQUIRE_FALSE(passes_overhang_filter(polar, deg2rad(90.)));
}

TEST_CASE("A 60 degree slope follows the threshold", "[SLAOverhangFilter]")
{
    const double polar = polar_for_slope(60.);

    // critical 20 deg -> accepts slopes up to 70 deg, so 60 gets in.
    REQUIRE(passes_overhang_filter(polar, deg2rad(20.)));

    // critical 45 deg -> accepts slopes up to 45 deg, so 60 is rejected.
    REQUIRE_FALSE(passes_overhang_filter(polar, deg2rad(45.)));
}

TEST_CASE("Upward facing surfaces never pass", "[SLAOverhangFilter]")
{
    // Polar below PI/2 means the normal has an upward component. The zero
    // threshold is the most permissive setting there is and still rejects.
    for (double polar_deg : {0., 30., 60., 89.})
        REQUIRE_FALSE(passes_overhang_filter(deg2rad(polar_deg), deg2rad(0.)));
}

TEST_CASE("The threshold is monotone in the surfaces it admits", "[SLAOverhangFilter]")
{
    // Raising the critical angle can only ever remove surfaces, never add
    // them. This is the property the DS-Online hint text promises ("the lower
    // the value, the more supports are generated").
    for (double slope_deg = 0.; slope_deg <= 90.; slope_deg += 5.) {
        const double polar = polar_for_slope(slope_deg);
        bool prev = true;
        for (double crit_deg = 0.; crit_deg <= 90.; crit_deg += 5.) {
            const bool now = passes_overhang_filter(polar, deg2rad(crit_deg));
            REQUIRE_FALSE((now && !prev)); // once rejected, stays rejected
            prev = now;
        }
    }
}

TEST_CASE("The predicate agrees with its closed form away from the boundary",
          "[SLAOverhangFilter]")
{
    // polar = acos(n.z) for a unit normal, so
    //     polar >= PI/2 + t   <=>   n.z <= -sin(t)
    // is an algebraically equivalent test. It is only a CROSS-CHECK, never the
    // implementation: acos and sin need not agree on the last ulp right at the
    // boundary, so samples that land within EPS of it are skipped.
    constexpr double EPS = 1e-6;

    std::mt19937 gen(0xC0FFEE); // fixed seed - a failure has to be reproducible
    std::uniform_real_distribution<double> nz_dis(-1., 1.);
    std::uniform_real_distribution<double> crit_dis(0., 90.);

    for (int i = 0; i < 20000; ++i) {
        const double nz    = nz_dis(gen);
        const double t     = deg2rad(crit_dis(gen));
        const double polar = std::acos(nz);

        if (std::abs(polar - (PI_ / 2. + t)) < EPS)
            continue; // too close to the boundary to compare fairly

        REQUIRE(passes_overhang_filter(polar, t) == (nz <= -std::sin(t)));
    }
}

TEST_CASE("The predicate reads nothing but its two arguments", "[SLAOverhangFilter]")
{
    // Calling it must not require a SupportTreeConfig or an SLAPrintObjectConfig
    // to exist. If this file compiles at all, that already holds - the test
    // body only pins the pure-function behaviour: same inputs, same answer.
    const double polar = polar_for_slope(30.);
    const double t     = deg2rad(45.);

    const bool first = passes_overhang_filter(polar, t);
    for (int i = 0; i < 8; ++i)
        REQUIRE(passes_overhang_filter(polar, t) == first);
}

// ─── Pipeline level: the filter actually runs, and it follows the threshold ───
//
// The cases above pin the predicate. These pin the wiring: that Phase 3 in
// support_points() really applies it, and really reads support_critical_angle.
//
// Each run stops at slaposSupportPoints - the same set_task() trick the CLI's
// --export-support-points uses - so no tree, no pad, no rasterization.

namespace {

// Pinned rather than left to the preset default because the count of support
// points depends on the slice grid, and every expectation below was measured at
// this pitch (it is also the SUPPORT_DETECTION_LAYER_HEIGHT the backend uses).
constexpr double DETECTION_LAYER_HEIGHT = 0.15;

// Runs `model_name`.obj as far as slaposSupportPoints and counts what came out.
// The caller has already put whatever it is varying into `fullcfg`; everything
// set here is the fixed harness every case shares.
size_t run_to_support_points(const std::string &model_name,
                             SLAFullPrintConfig &fullcfg)
{
    Model model = FileReader::load_model(TEST_DATA_DIR PATH_SEPARATOR
                                         + model_name + ".obj");

    fullcfg.printer_technology.setInt(ptSLA);
    fullcfg.supports_enable.value = true;  // support_points() returns early without it
    fullcfg.pad_enable.value      = false; // nothing here reaches the pad step anyway
    fullcfg.set("layer_height", DETECTION_LAYER_HEIGHT);
    fullcfg.set("initial_layer_height", DETECTION_LAYER_HEIGHT);

    DynamicPrintConfig cfg;
    cfg.apply(fullcfg);

    SLAPrint print;
    print.set_status_callback([](const PrintBase::SlicingStatus &) {});
    print.apply(model, cfg);

    PrintBase::TaskParams task;
    task.to_object_step = slaposSupportPoints;
    print.set_task(task);
    print.process();

    REQUIRE_FALSE(print.objects().empty());
    return print.objects().front()->get_support_points().size();
}

// Support points produced for `model_name`.obj at the given overhang threshold,
// on the default tree. branchingsupport_critical_angle is left at its preset
// value on purpose - the Default tree must not read it.
size_t support_point_count(const std::string &model_name, double critical_angle_deg)
{
    SLAFullPrintConfig fullcfg;
    fullcfg.support_critical_angle.value = critical_angle_deg;
    return run_to_support_points(model_name, fullcfg);
}

// The same, with the tree type and BOTH critical angles pinned.
size_t support_point_count(const std::string  &model_name,
                           sla::SupportTreeType tree,
                           double               critical_angle_deg,
                           double               branching_critical_angle_deg)
{
    SLAFullPrintConfig fullcfg;
    fullcfg.support_tree_type.value                = tree;
    fullcfg.support_critical_angle.value           = critical_angle_deg;
    fullcfg.branchingsupport_critical_angle.value  = branching_critical_angle_deg;
    return run_to_support_points(model_name, fullcfg);
}

} // namespace

TEST_CASE("Raising the overhang threshold removes support points",
          "[SLAOverhangFilter][SLASupportGeneration]")
{
    // V_standing.obj is the fixture for this because 83% of its down-facing
    // area sits at a slope of 45-75 degrees from horizontal - straddling the
    // default threshold, so the three settings cannot land on the same answer.
    const size_t n0  = support_point_count("V_standing", 0.);
    const size_t n45 = support_point_count("V_standing", 45.);
    const size_t n90 = support_point_count("V_standing", 90.);

    INFO("V_standing point counts: 0deg=" << n0 << " 45deg=" << n45
                                          << " 90deg=" << n90);

    // A zero threshold admits every overhang, so there has to be something to
    // take away in the first place; without this the chain below would pass
    // trivially on 0 > 0 > 0.
    REQUIRE(n0 > 0);

    // Strictly decreasing: a larger support_critical_angle supports FEWER
    // surfaces. This is the direction frozen by
    // capability sla-overhang-threshold-semantics and promised by the
    // DS-Online hint text ("the lower the value, the more supports").
    REQUIRE(n0 > n45);
    REQUIRE(n45 > n90);
}

TEST_CASE("Horizontal down-facing surfaces survive up to the 45 degree tie",
          "[SLAOverhangFilter][SLASupportGeneration]")
{
    // U_overhang.obj is the opposite fixture: 100% of its down-facing area is
    // horizontal, so at the default setting the filter must not touch it.
    //
    // It does NOT stay invariant all the way to 90 degrees, and that is the
    // engine behaving as it always has - not a defect in the filter. Two
    // independent 0.2 mm radii happen to be the same number:
    //
    //   the sampler   SampleConfigFactory.cpp:86
    //                 minimal_distance_from_outline = head_radius   (0.2 mm)
    //   get_normal()  MeshNormals.cpp - averages the neighbouring faces
    //                 whenever the point sits within eps of an edge, with
    //                 eps = head_front_radius_mm                    (0.2 mm)
    //
    // So every point the sampler insets from an island outline lands exactly on
    // get_normal()'s `squaredDistance < eps*eps` knife edge, decided by the last
    // bits of the float32 SupportPoint::pos. On this fixture 8 of the 20 points
    // fall inside it: their flat (0,0,-1) normal gets averaged with the vertical
    // side wall, giving polar == 135 deg instead of 180 deg.
    //
    // 135 deg passes while `90 + t <= 135`, i.e. while t <= 45 - which is why
    // the default is unaffected and 90 degrees is not. Step 6 computes the same
    // normals with the same eps, so those 8 points were already dropped there
    // before this change; Phase 3 only makes the drop visible earlier.
    // Documented as a known limitation in design.md R7 and in capability
    // sla-overhang-threshold-semantics.
    const size_t n0  = support_point_count("U_overhang", 0.);
    const size_t n45 = support_point_count("U_overhang", 45.);
    const size_t n90 = support_point_count("U_overhang", 90.);

    INFO("U_overhang point counts: 0deg=" << n0 << " 45deg=" << n45
                                          << " 90deg=" << n90);

    // Up to and including the default, a flat overhang keeps every point. The
    // absolute value is the reading taken from the CLI at this layer height
    // before the filter existed (openspec change
    // align-support-point-overhang-filter, task 0.3).
    REQUIRE(n0 == 20u);
    REQUIRE(n45 == 20u);

    // Past the tie the edge-blended points go. Monotonicity still holds, and
    // the 12 interior points - far enough from any outline for get_normal() to
    // return the true face normal - survive the maximum threshold.
    REQUIRE(n90 <= n45);
    REQUIRE(n90 == 12u);
}

// ─── Task 2.R.4: the two stages must agree on the normal tolerance ───────────
//
// Phase 3 hands normals() its own eps and its own threshold, worked out from a
// switch of its own in SLAPrintSteps.cpp. The support tree gets its eps
// (head_front_radius_mm) and its threshold (overhang_angle_threshold) from
// make_support_cfg(). If those two switches ever disagree, the same point gets
// a different normal in each stage and the shared predicate cannot save it -
// step 5 would pass a point that step 6 then throws away, which is exactly the
// orphan-point defect this change exists to remove.

namespace {

struct Phase3Inputs
{
    double overhang_threshold_rad;
    double normal_eps_mm;
};

// A VERBATIM COPY of the second switch in SLAPrintSteps.cpp's support_points().
//
// It is a replica, not the real thing: those are function-local doubles with no
// accessor, so this test pins the FORMULA and its tree-type grouping, not the
// running variable. Editing SLAPrintSteps.cpp without editing this replica will
// NOT be caught here - keep the two in sync by hand.
Phase3Inputs phase3_inputs_replica(const SLAPrintObjectConfig &c)
{
    Phase3Inputs in{0., 0.};

    switch (c.support_tree_type) {
    case sla::SupportTreeType::Default:
        in.overhang_threshold_rad = c.support_critical_angle.getFloat() * PI / 180.0;
        in.normal_eps_mm          = 0.5 * c.support_head_front_diameter.getFloat();
        break;
    case sla::SupportTreeType::Branching:
    case sla::SupportTreeType::Organic:
        in.overhang_threshold_rad = c.branchingsupport_critical_angle.getFloat() * PI / 180.0;
        in.normal_eps_mm          = 0.5 * c.branchingsupport_head_front_diameter.getFloat();
        break;
    }

    return in;
}

// Values chosen so that no two of them coincide: a switch that reads the wrong
// key cannot pass by accident.
SLAPrintObjectConfig divergent_config()
{
    SLAPrintObjectConfig c;
    c.support_head_front_diameter.value           = 0.4;
    c.branchingsupport_head_front_diameter.value  = 0.6;
    c.support_critical_angle.value                = 20.;
    c.branchingsupport_critical_angle.value       = 70.;
    return c;
}

} // namespace

TEST_CASE("Phase 3 and the support tree derive the same tolerance and threshold",
          "[SLAOverhangFilter]")
{
    SLAPrintObjectConfig c = divergent_config();

    for (auto tree : {sla::SupportTreeType::Default,
                      sla::SupportTreeType::Branching,
                      sla::SupportTreeType::Organic}) {
        c.support_tree_type.value = tree;

        const Phase3Inputs           phase3 = phase3_inputs_replica(c);
        const sla::SupportTreeConfig scfg   = make_support_cfg(c);

        INFO("support_tree_type = " << int(tree));

        // Exact equality, not Approx: both sides are the same expression over
        // the same double, so any difference at all means the two switches have
        // drifted apart - which is the defect, however small the gap.
        REQUIRE(phase3.normal_eps_mm == scfg.head_front_radius_mm);
        REQUIRE(phase3.overhang_threshold_rad == scfg.overhang_angle_threshold);
    }
}

TEST_CASE("The tolerance and threshold really do change with the tree type",
          "[SLAOverhangFilter]")
{
    // Without this the case above could pass on a switch that ignores the tree
    // type entirely, as long as BOTH sides ignored it the same way.
    SLAPrintObjectConfig c = divergent_config();

    c.support_tree_type.value = sla::SupportTreeType::Default;
    const Phase3Inputs def = phase3_inputs_replica(c);

    c.support_tree_type.value = sla::SupportTreeType::Branching;
    const Phase3Inputs brn = phase3_inputs_replica(c);

    c.support_tree_type.value = sla::SupportTreeType::Organic;
    const Phase3Inputs org = phase3_inputs_replica(c);

    REQUIRE(def.normal_eps_mm == 0.2);          // 0.5 * 0.4
    REQUIRE(brn.normal_eps_mm == 0.3);          // 0.5 * 0.6
    REQUIRE(def.overhang_threshold_rad == 20. * PI / 180.0);
    REQUIRE(brn.overhang_threshold_rad == 70. * PI / 180.0);

    // Organic groups with Branching, NOT with Default. make_support_cfg() puts
    // Branching and Organic in one case via [[fallthrough]], while the
    // head_diameter switch a few lines above Phase 3 groups Organic with
    // Default instead. Copying that other grouping here is a real defect that
    // was made and fixed once already (design.md D3); this is the guard.
    REQUIRE(org.normal_eps_mm == brn.normal_eps_mm);
    REQUIRE(org.overhang_threshold_rad == brn.overhang_threshold_rad);
    REQUIRE(org.normal_eps_mm != def.normal_eps_mm);
    REQUIRE(org.overhang_threshold_rad != def.overhang_threshold_rad);
}

// ─── Task 2.R.5: each tree type is steered by its own critical angle ─────────
//
// The case above works on the config alone. This one runs the pipeline, so it
// pins that Phase 3 actually consumes the value the dispatch produced.
//
// Counts are never compared ACROSS tree types - the sampler's head_diameter is
// itself tree-type dependent, so two tree types start from different point
// sets. Within one tree type the sampling is fixed, and only the angle moves.

namespace {

// Raising the governing key must remove points; raising the other one must
// change nothing at all.
void require_governed_by(sla::SupportTreeType tree, bool branching_key_governs)
{
    constexpr double LOW  = 20.; // permissive: admits slopes up to 70 deg
    constexpr double HIGH = 70.; // restrictive: admits slopes up to 20 deg

    //                                              (support, branchingsupport)
    const size_t both_low  = support_point_count("V_standing", tree, LOW,  LOW);
    const size_t def_high  = support_point_count("V_standing", tree, HIGH, LOW);
    const size_t brn_high  = support_point_count("V_standing", tree, LOW,  HIGH);

    INFO("tree=" << int(tree) << " both_low=" << both_low
                 << " support_critical_angle raised=" << def_high
                 << " branchingsupport_critical_angle raised=" << brn_high);

    // Guard against a vacuous pass on 0 == 0 == 0.
    REQUIRE(both_low > 0);

    if (branching_key_governs) {
        REQUIRE(brn_high < both_low);   // the branching key bites
        REQUIRE(def_high == both_low);  // the default key is inert
    } else {
        REQUIRE(def_high < both_low);   // the default key bites
        REQUIRE(brn_high == both_low);  // the branching key is inert
    }
}

} // namespace

TEST_CASE("The default tree is steered by support_critical_angle",
          "[SLAOverhangFilter][SLASupportGeneration]")
{
    require_governed_by(sla::SupportTreeType::Default, /*branching_key_governs=*/false);
}

TEST_CASE("The branching tree is steered by branchingsupport_critical_angle",
          "[SLAOverhangFilter][SLASupportGeneration]")
{
    require_governed_by(sla::SupportTreeType::Branching, /*branching_key_governs=*/true);
}

TEST_CASE("The organic tree is steered by branchingsupport_critical_angle too",
          "[SLAOverhangFilter][SLASupportGeneration]")
{
    // The case that would have caught the grouping defect at pipeline level.
    require_governed_by(sla::SupportTreeType::Organic, /*branching_key_governs=*/true);
}

// --- Tasks 3.R.2 / 3.R.3: step 6 no longer gates on the overhang angle ------
//
// Two things had to be checked before any of this could be written, and both
// changed the shape of the tests:
//
// 1. There is no way to reach a SupportTreeBuilder through SLAPrint.
//    SLAPrintObject::SupportData::create_support_tree() calls
//    sla::create_support_tree(), which returns a bare indexed_triangle_set and
//    throws the builder away. Head counts therefore have to come from driving
//    DefaultSupportTree::execute() directly - the same entry point step 6 uses
//    for the default tree, and the pattern sla_per_point_geometry_tests.cpp
//    already relies on.
//
// 2. The built MESH is not bitwise deterministic. DefaultSupportTree.cpp runs
//    its loops on suptree_ex_policy, a hard-coded ex_tbb
//    (DefaultSupportTree.hpp:28) with no sequential override. Heads themselves
//    ARE deterministic - the parallel loop fills a pre-sized vector by index
//    and a sequential loop then calls add_head() in index order
//    (DefaultSupportTree.cpp:589-601) - but pillar routing and bridging run in
//    parallel and may append in any order. So these tests compare HEADS and
//    structure COUNTS, never raw vertex buffers.

namespace {

// An apex-down cone: apex at z = 0, base circle of radius `r` at z = h. Its
// lateral surface is a down-facing overhang at a slope of atan(h/r) from
// horizontal, which is the steepness the removed gate used to reject.
indexed_triangle_set apex_down_cone(double r, double h)
{
    indexed_triangle_set c = its_make_cone(r, h);
    for (Vec3f &v : c.vertices)
        v.z() = float(h) - v.z();
    for (Vec3i32 &f : c.indices)
        std::swap(f(1), f(2)); // keep the normals pointing outwards
    return c;
}

// r = 5, h = 10 puts the lateral surface at atan(10/5) = 63.43 deg of slope,
// so the normal's polar angle is 180 - 63.43 = 116.57 deg. The gate that used
// to live in add_pinheads() passed a point only when polar >= 90 + threshold,
// i.e. only for a threshold below 26.57 deg. At the 45 and 90 degree settings
// below it would have rejected this point outright, which is exactly what must
// no longer happen.
constexpr double CONE_R = 5.;
constexpr double CONE_H = 10.;

// A point on that lateral surface at half height, where the cone's radius is
// CONE_R * 0.5.
sla::SupportPoint cone_wall_point(const sla::SupportTreeConfig &cfg,
                                  sla::SupportPointType         type)
{
    sla::SupportPoint sp;
    sp.pos               = Vec3f(float(CONE_R * 0.5), 0.f, float(CONE_H * 0.5));
    sp.head_front_radius = float(cfg.head_front_radius_mm);
    sp.type              = type;
    return sp;
}

const sla::Head *head_of(const sla::SupportTreeBuilder &b, long pt_index)
{
    auto it = std::find_if(b.heads().begin(), b.heads().end(),
                           [pt_index](const sla::Head &h) {
                               return h.is_valid() && h.id == pt_index;
                           });
    return it == b.heads().end() ? nullptr : &*it;
}

size_t valid_head_count(const sla::SupportTreeBuilder &b)
{
    return size_t(std::count_if(b.heads().begin(), b.heads().end(),
                                [](const sla::Head &h) { return h.is_valid(); }));
}

} // namespace

TEST_CASE("The support tree ignores the overhang threshold entirely",
          "[SLAOverhangFilter][SLASupportGeneration]")
{
    // The heart of task 3.R.2. One steep point, three wildly different
    // thresholds, and the tree must not care about any of them - the filtering
    // already happened upstream, in step 5.
    const indexed_triangle_set cone = apex_down_cone(CONE_R, CONE_H);

    size_t heads[3] = {0, 0, 0};
    int    i        = 0;

    for (double threshold_deg : {0., 45., 90.}) {
        sla::SupportTreeConfig cfg;
        cfg.overhang_angle_threshold = threshold_deg * PI / 180.0;

        sla::SupportableMesh sm{cone,
                                {cone_wall_point(cfg, sla::SupportPointType::slope)},
                                cfg};
        sla::SupportTreeBuilder builder;
        sla::DefaultSupportTree::execute(builder, sm);

        INFO("overhang threshold " << threshold_deg << " deg");

        // Not merely "the same count" - the point must actually carry a head.
        // A run where every threshold produced nothing would satisfy equality
        // while proving the opposite of what is claimed here.
        REQUIRE(head_of(builder, 0) != nullptr);

        heads[i++] = valid_head_count(builder);
    }

    INFO("valid head counts: 0deg=" << heads[0] << " 45deg=" << heads[1]
                                    << " 90deg=" << heads[2]);
    REQUIRE(heads[0] > 0);
    REQUIRE(heads[0] == heads[1]);
    REQUIRE(heads[1] == heads[2]);
}

TEST_CASE("An imported point list survives every overhang threshold",
          "[SLAOverhangFilter][SLASupportGeneration]")
{
    // The other half of task 3.R.2, end to end this time. Setting
    // sla_points_status to UserModified before apply() is exactly what
    // --import-support-points does (CLI/ProcessActions.cpp:649-650), and it
    // makes support_points() copy the list through and return before Phase 3
    // ever runs. Changing the angle must not move the count.
    size_t counts[3] = {0, 0, 0};

    for (size_t idx = 0; idx < 3; ++idx) {
        const double crit_deg = (idx == 0) ? 0. : (idx == 1) ? 45. : 90.;

        Model model = FileReader::load_model(TEST_DATA_DIR PATH_SEPARATOR
                                             "U_overhang.obj");
        REQUIRE_FALSE(model.objects.empty());

        // Three hand-placed points, deliberately including two on the steep
        // side walls that the old step-6 gate would have thrown away.
        sla::SupportPoints pts;
        for (const Vec3f &p : {Vec3f(5.f, 5.f, 0.f),
                               Vec3f(0.f, 5.f, 5.f),
                               Vec3f(10.f, 5.f, 5.f)}) {
            sla::SupportPoint sp;
            sp.pos               = p;
            sp.head_front_radius = 0.2f;
            sp.type              = sla::SupportPointType::manual_add;
            pts.push_back(sp);
        }

        ModelObject *mo        = model.objects.front();
        mo->sla_support_points = pts;
        mo->sla_points_status  = sla::PointsStatus::UserModified;

        SLAFullPrintConfig fullcfg;
        fullcfg.printer_technology.setInt(ptSLA);
        fullcfg.supports_enable.value        = true;
        fullcfg.pad_enable.value             = false;
        fullcfg.support_critical_angle.value = crit_deg;
        fullcfg.set("layer_height", DETECTION_LAYER_HEIGHT);
        fullcfg.set("initial_layer_height", DETECTION_LAYER_HEIGHT);

        DynamicPrintConfig cfg;
        cfg.apply(fullcfg);

        SLAPrint print;
        print.set_status_callback([](const PrintBase::SlicingStatus &) {});
        print.apply(model, cfg);

        PrintBase::TaskParams task;
        task.to_object_step = slaposSupportTree; // all the way through step 6
        print.set_task(task);
        print.process();

        REQUIRE_FALSE(print.objects().empty());
        counts[idx] = print.objects().front()->get_support_points().size();
    }

    INFO("imported point counts: 0deg=" << counts[0] << " 45deg=" << counts[1]
                                        << " 90deg=" << counts[2]);

    // All three points survive, at every setting. Nothing about the angle
    // slider may touch a list the caller supplied.
    REQUIRE(counts[0] == 3u);
    REQUIRE(counts[1] == 3u);
    REQUIRE(counts[2] == 3u);
}

TEST_CASE("SupportPointType no longer changes what the tree builds",
          "[SLAOverhangFilter][SLASupportGeneration]")
{
    // Task 3.R.3. With the gate gone, `type` is a provenance label for the
    // front end and nothing more: two point sets identical except for it must
    // produce the same structures.
    //
    // Compared at head level, not on the meshes - see the note at the top of
    // this section for why raw vertex buffers cannot be compared here.
    const indexed_triangle_set cone = apex_down_cone(CONE_R, CONE_H);

    auto build = [&cone](sla::SupportPointType type) {
        sla::SupportTreeConfig cfg;
        sla::SupportableMesh   sm{cone, {cone_wall_point(cfg, type)}, cfg};
        auto builder = std::make_unique<sla::SupportTreeBuilder>();
        sla::DefaultSupportTree::execute(*builder, sm);
        return builder;
    };

    auto manual = build(sla::SupportPointType::manual_add);
    auto slope  = build(sla::SupportPointType::slope);

    REQUIRE(valid_head_count(*manual) > 0);
    REQUIRE(valid_head_count(*manual) == valid_head_count(*slope));

    const sla::Head *hm = head_of(*manual, 0);
    const sla::Head *hs = head_of(*slope, 0);
    REQUIRE(hm);
    REQUIRE(hs);

    // Exact equality: both heads come from the same arithmetic on the same
    // inputs, so any difference at all means `type` reached a branch it should
    // no longer reach.
    REQUIRE(hm->pos == hs->pos);
    REQUIRE(hm->dir == hs->dir);
    REQUIRE(hm->r_back_mm == hs->r_back_mm);
    REQUIRE(hm->r_pin_mm == hs->r_pin_mm);
    REQUIRE(hm->width_mm == hs->width_mm);
    REQUIRE(hm->penetration_mm == hs->penetration_mm);

    // Counts of the downstream structures, which are order independent even
    // though the order they were appended in is not.
    REQUIRE(manual->pillars().size() == slope->pillars().size());
    REQUIRE(manual->pedestals().size() == slope->pedestals().size());
    REQUIRE(manual->bridges().size() == slope->bridges().size());
}

// --- Tasks 3.R.4 / 3.R.5: what the tree still refuses, and what it now allows -
//
// Removing the angle gate exempted every incoming point from the OVERHANG
// threshold and nothing else. The physical limits below it are untouched, and
// these cases pin that: a point can still be refused for a normal that points
// the wrong way, and still refused for having nowhere to put a head. What
// changed is that a vertical wall - which the old gate rejected outright at the
// default setting - now grows a head, slanting out at the bracing angle.

namespace {

// A plain cylinder: flat down-facing bottom at z = 0, flat up-facing top at
// z = h, vertical side wall at radius r.
indexed_triangle_set upright_cylinder(double r, double h)
{
    return its_make_cylinder(r, h);
}

// Two of those stacked with a `gap` mm slit between them: the upper one spans
// z = [0, h], the lower one z = [-h-gap, -gap]. A point on the underside of the
// upper disc has `gap` mm of clearance and nothing else.
indexed_triangle_set pinched_slit(double r, double h, double gap)
{
    indexed_triangle_set upper = its_make_cylinder(r, h);
    indexed_triangle_set lower = its_make_cylinder(r, h);

    const float dz = float(-(h + gap));
    for (Vec3f &v : lower.vertices)
        v.z() += dz;

    const int voff = int(upper.vertices.size());
    upper.vertices.insert(upper.vertices.end(),
                          lower.vertices.begin(), lower.vertices.end());
    for (const Vec3i32 &f : lower.indices)
        upper.indices.emplace_back(f(0) + voff, f(1) + voff, f(2) + voff);

    return upper;
}

double head_polar(const sla::Head &h)
{
    return std::get<0>(sla::dir_to_spheric(h.dir));
}

sla::SupportPoint point_at(const Vec3f &pos, const sla::SupportTreeConfig &cfg)
{
    sla::SupportPoint sp;
    sp.pos               = pos;
    sp.head_front_radius = float(cfg.head_front_radius_mm);
    sp.type              = sla::SupportPointType::manual_add;
    return sp;
}

constexpr double HeadAngleTol = 1e-3; // rad

} // namespace

TEST_CASE("An upward facing point is still refused by normal_cutoff_angle",
          "[SLAOverhangFilter][SLASupportGeneration]")
{
    // Task 3.R.4, first half. normal_cutoff_angle is 150 deg, and the surviving
    // gate reads `if (polar < PI - normal_cutoff_angle) return;` - so a normal
    // within 30 deg of straight up is thrown out. That gate was NOT removed,
    // and exempting imported points from the overhang angle must not have
    // exempted them from this one too.
    const double CYL_R = 8., CYL_H = 5.;
    const indexed_triangle_set cyl = upright_cylinder(CYL_R, CYL_H);

    sla::SupportTreeConfig cfg;

    // 0: the flat underside, normal straight down (polar 180) - the control.
    // 1: the flat top, normal straight up (polar 0) - must be refused.
    sla::SupportPoints pts{point_at(Vec3f(0.f, 0.f, 0.f), cfg),
                           point_at(Vec3f(0.f, 0.f, float(CYL_H)), cfg)};

    sla::SupportableMesh   sm{cyl, pts, cfg};
    sla::SupportTreeBuilder builder;
    sla::DefaultSupportTree::execute(builder, sm);

    // Without this the case below would pass on a build that produced nothing
    // at all, which proves the opposite of what is claimed.
    REQUIRE(head_of(builder, 0) != nullptr);

    REQUIRE(head_of(builder, 1) == nullptr);
}

TEST_CASE("A point with nowhere to put a head fails quietly",
          "[SLAOverhangFilter][SLASupportGeneration]")
{
    // Task 3.R.4, second half. A head needs roughly
    //   head_width + 2*head_back_radius + 2*head_front_radius - penetration
    //   = 1.0 + 1.0 + 0.4 - 0.5 = 1.9 mm
    // of clear space along its direction, and the mini-pillar fallback still
    // needs 2 * head_fallback_radius = 0.5 mm. A 0.3 mm slit gives neither.
    //
    // The requirement is that this is a QUIET refusal: no head, and no
    // exception escaping the build. A point the caller placed somewhere
    // impossible must not take the whole slicing job down with it.
    const double CYL_R = 8., CYL_H = 5., GAP = 0.3;
    const indexed_triangle_set slit = pinched_slit(CYL_R, CYL_H, GAP);

    sla::SupportTreeConfig cfg;

    // Dead centre of the underside, 8 mm from the nearest rim, so tilting the
    // head sideways cannot escape the slit either.
    sla::SupportableMesh sm{slit, {point_at(Vec3f(0.f, 0.f, 0.f), cfg)}, cfg};

    sla::SupportTreeBuilder builder;
    REQUIRE_NOTHROW(sla::DefaultSupportTree::execute(builder, sm));

    REQUIRE(head_of(builder, 0) == nullptr);
}

TEST_CASE("A point on a vertical wall now grows a slanted head",
          "[SLAOverhangFilter][SLASupportGeneration]")
{
    // Task 3.R.5, and the clearest demonstration of what this change bought.
    //
    // A cylinder's side wall is vertical, so its normal is horizontal and
    // polar = 90 deg. The removed gate passed a point only when
    // polar >= 90 + threshold, so at the default 45 deg setting this point used
    // to be dropped without a word - the orphan-point defect, seen from the
    // tree's side. It must now carry a head.
    //
    // The head does not point sideways, though: the surviving saturation
    // `polar = max(polar, PI - pt_slope)` pushes it to PI - bridge_slope,
    // i.e. 135 deg, so the pillar slants down and out at the bracing angle.
    // That is risk R2 in design.md, pinned here as intended behaviour rather
    // than left as prose.
    const double CYL_R = 8., CYL_H = 5.;
    const indexed_triangle_set cyl = upright_cylinder(CYL_R, CYL_H);

    sla::SupportTreeConfig cfg;

    // Halfway up the side wall. Below it is the object elevation - 10 mm of
    // clear air by default - so the pillar has somewhere to go.
    sla::SupportableMesh sm{cyl,
                            {point_at(Vec3f(float(CYL_R), 0.f, float(CYL_H * 0.5)), cfg)},
                            cfg};

    sla::SupportTreeBuilder builder;
    sla::DefaultSupportTree::execute(builder, sm);

    const sla::Head *h = head_of(builder, 0);
    REQUIRE(h);

    const double polar = head_polar(*h);
    INFO("head polar = " << polar * 180. / PI << " deg");

    // Saturated to the bracing angle, not left at the wall's own 90 deg.
    REQUIRE(std::abs(polar - (PI - cfg.bridge_slope)) < HeadAngleTol);

    // And genuinely slanted - not straight down, which is what a flat overhang
    // would have produced and would make this test prove nothing.
    REQUIRE(std::abs(polar - PI) > HeadAngleTol);
}

// --- Task 4.R.2: changing a critical angle invalidates the POINTS step ------
//
// The filter moved to slaposSupportPoints, so the angle is now an input to
// which points EXIST, not just to how the tree is grown. If only
// slaposSupportTree were invalidated, a caller that keeps an SLAPrintObject
// alive across a config edit - the GUI, not the backend's one-shot CLI - would
// rebuild the tree from a stale, differently-filtered point list.
//
// The registration lives in SLAPrint.cpp's invalidate_state_by_config_options().
// It is an else-if chain, so the two keys are matched by the slaposSupportPoints
// branch and never reach the slaposSupportTree branch below it; the tree still
// gets invalidated because invalidate_step(slaposSupportPoints) propagates.

namespace {

struct InvalidationProbe
{
    PrintBase::ApplyStatus status      = PrintBase::APPLY_STATUS_UNCHANGED;
    bool                   points_done = false;
    bool                   tree_done   = false;
    bool                   same_object = false;
};

SLAFullPrintConfig invalidation_base_config()
{
    SLAFullPrintConfig c;
    c.printer_technology.setInt(ptSLA);
    c.supports_enable.value = true;
    c.pad_enable.value      = false;
    // Both angles pinned so a case that moves one is moving it off a KNOWN
    // value, not off whatever the preset happened to carry.
    c.support_critical_angle.value              = 45.;
    c.branchingsupport_critical_angle.value     = 45.;
    c.set("layer_height", DETECTION_LAYER_HEIGHT);
    c.set("initial_layer_height", DETECTION_LAYER_HEIGHT);
    return c;
}

// Runs U_overhang.obj through step 6, applies a config with `tweak` on top of
// the same base, and reports what that did to the two steps.
InvalidationProbe probe_invalidation(const std::function<void(SLAFullPrintConfig &)> &tweak)
{
    Model model = FileReader::load_model(TEST_DATA_DIR PATH_SEPARATOR "U_overhang.obj");

    SLAPrint print;
    print.set_status_callback([](const PrintBase::SlicingStatus &) {});

    DynamicPrintConfig cfg;
    cfg.apply(invalidation_base_config());
    print.apply(model, cfg);

    PrintBase::TaskParams task;
    task.to_object_step = slaposSupportTree; // both steps under test end up done
    print.set_task(task);
    print.process();

    REQUIRE_FALSE(print.objects().empty());
    const SLAPrintObject *before = print.objects().front();

    // The starting state has to be "done", or the assertions after the second
    // apply() would hold for a print that never ran.
    REQUIRE(before->is_step_done(slaposSupportPoints));
    REQUIRE(before->is_step_done(slaposSupportTree));

    SLAFullPrintConfig changed = invalidation_base_config();
    tweak(changed);
    DynamicPrintConfig cfg2;
    cfg2.apply(changed);

    InvalidationProbe out;
    out.status = print.apply(model, cfg2);

    REQUIRE_FALSE(print.objects().empty());
    const SLAPrintObject *after = print.objects().front();

    // If apply() threw the object away and built a fresh one, every step would
    // read as not-done for a reason that has nothing to do with the
    // registration under test. Reported rather than asserted here so each case
    // states it for itself.
    out.same_object = (after == before);
    out.points_done = after->is_step_done(slaposSupportPoints);
    out.tree_done   = after->is_step_done(slaposSupportTree);
    return out;
}

} // namespace

TEST_CASE("Changing support_critical_angle invalidates the support point step",
          "[SLAOverhangFilter][SLASupportGeneration]")
{
    const InvalidationProbe p = probe_invalidation([](SLAFullPrintConfig &c) {
        c.support_critical_angle.value = 70.;
    });

    INFO("apply status = " << int(p.status) << " points_done=" << p.points_done
                           << " tree_done=" << p.tree_done);

    REQUIRE(p.same_object);
    REQUIRE(p.status == PrintBase::APPLY_STATUS_INVALIDATED);
    REQUIRE_FALSE(p.points_done);
    REQUIRE_FALSE(p.tree_done); // propagated from the point step
}

TEST_CASE("Changing branchingsupport_critical_angle invalidates the point step too",
          "[SLAOverhangFilter][SLASupportGeneration]")
{
    // Registered separately from its non-prefixed twin, and just as easy to
    // leave out - the key is only ever compared as a string, so a missing or
    // misspelled entry fails silently rather than failing to compile.
    const InvalidationProbe p = probe_invalidation([](SLAFullPrintConfig &c) {
        c.branchingsupport_critical_angle.value = 70.;
    });

    INFO("apply status = " << int(p.status) << " points_done=" << p.points_done
                           << " tree_done=" << p.tree_done);

    REQUIRE(p.same_object);
    REQUIRE(p.status == PrintBase::APPLY_STATUS_INVALIDATED);
    REQUIRE_FALSE(p.points_done);
    REQUIRE_FALSE(p.tree_done);
}

TEST_CASE("A tree-only setting still leaves the support point step alone",
          "[SLAOverhangFilter][SLASupportGeneration]")
{
    // The control, and the reason the two cases above are worth anything: if
    // apply() simply invalidated everything on any config change, they would
    // pass no matter how the keys were registered.
    //
    // support_base_height belongs to the slaposSupportTree branch and was left
    // there by this change. Moving it must reach step 6 and stop.
    const InvalidationProbe p = probe_invalidation([](SLAFullPrintConfig &c) {
        c.support_base_height.value = c.support_base_height.value + 0.5;
    });

    INFO("apply status = " << int(p.status) << " points_done=" << p.points_done
                           << " tree_done=" << p.tree_done);

    REQUIRE(p.same_object);
    REQUIRE(p.points_done);     // step 5 survives - the registration discriminates
    REQUIRE_FALSE(p.tree_done); // step 6 does not
}

// --- Tasks 5.1 / 5.2: the spec scenarios that nothing else pinned ----------
//
// Walking both delta specs scenario by scenario turned up three that no test
// in this file, or in sla_support_point_io_tests.cpp, actually covered. They
// are filled in below; the rest of the mapping is recorded in tasks.md.

namespace {

// PhrozenOrca's rule, written out so the divergence can be compared rather
// than described. That project tests the surface's slope against the setting
// directly - passes while `slope <= critical angle` - which is the OPPOSITE
// scale from this fork's `slope <= 90 - critical angle`.
//
// In polar terms, with slope = PI - polar:
//     theirs   PI - polar <= t   <=>   polar >= PI - t
//     ours                             polar >= PI/2 + t
// The two thresholds coincide exactly when PI - t == PI/2 + t, i.e. t == PI/4.
bool phrozen_orca_passes_overhang_filter(double polar, double threshold)
{
    return polar >= M_PI - threshold;
}

} // namespace

TEST_CASE("The two projects' predicates agree only at 45 degrees",
          "[SLAOverhangFilter]")
{
    // capability sla-overhang-threshold-semantics, scenario "45 度為唯一重合點".
    //
    // 45 is the web default, which is why the divergence has never bitten: at
    // that one setting both projects answer identically for every surface. The
    // moment a caller moves the slider the two disagree, and a merge that took
    // either predicate verbatim would silently flip the meaning of the number.

    // At 45 degrees: identical verdicts, all the way round.
    for (double slope_deg = 0.; slope_deg <= 90.; slope_deg += 1.) {
        const double polar = polar_for_slope(slope_deg);
        INFO("slope " << slope_deg << " deg at a 45 degree setting");
        REQUIRE(passes_overhang_filter(polar, deg2rad(45.))
                == phrozen_orca_passes_overhang_filter(polar, deg2rad(45.)));
    }

    // At every other setting: there is a surface they disagree about. Without
    // this half the case above would also pass on two identical predicates.
    for (double crit_deg : {0., 10., 20., 30., 40., 50., 60., 70., 80., 90.}) {
        bool found_disagreement = false;
        for (double slope_deg = 0.; slope_deg <= 90. && !found_disagreement;
             slope_deg += 1.) {
            const double polar = polar_for_slope(slope_deg);
            found_disagreement =
                passes_overhang_filter(polar, deg2rad(crit_deg))
                != phrozen_orca_passes_overhang_filter(polar, deg2rad(crit_deg));
        }
        INFO("setting " << crit_deg << " deg");
        REQUIRE(found_disagreement);
    }
}

// --- Points the caller carried in, on the AUTOMATIC path -------------------
//
// Distinct from the imported (UserModified) path already covered above.
// prepare_permanent_support_points() (SLAPrintSteps.cpp) copies every
// manual_add point out of ModelObject::sla_support_points while automatic
// detection still runs, and support_points() merges them in AFTER Phase 3 -
// which is the whole reason the filter had to go where design.md D2 put it.

namespace {

// Support points produced for U_overhang.obj at `critical_angle_deg`, with
// `carried` planted on the model object first. Leaving sla_points_status alone
// keeps the automatic path live, so these ride alongside generated points
// rather than replacing them.
size_t support_point_count_with_carried(double critical_angle_deg,
                                        const sla::SupportPoints &carried)
{
    Model model = FileReader::load_model(TEST_DATA_DIR PATH_SEPARATOR
                                         "U_overhang.obj");
    REQUIRE_FALSE(model.objects.empty());
    model.objects.front()->sla_support_points = carried;

    SLAFullPrintConfig fullcfg;
    fullcfg.printer_technology.setInt(ptSLA);
    fullcfg.supports_enable.value        = true;
    fullcfg.pad_enable.value             = false;
    fullcfg.support_critical_angle.value = critical_angle_deg;
    fullcfg.set("layer_height", DETECTION_LAYER_HEIGHT);
    fullcfg.set("initial_layer_height", DETECTION_LAYER_HEIGHT);

    DynamicPrintConfig cfg;
    cfg.apply(fullcfg);

    SLAPrint print;
    print.set_status_callback([](const PrintBase::SlicingStatus &) {});
    print.apply(model, cfg);

    PrintBase::TaskParams task;
    task.to_object_step = slaposSupportPoints;
    print.set_task(task);
    print.process();

    REQUIRE_FALSE(print.objects().empty());
    return print.objects().front()->get_support_points().size();
}

// Two points on U_overhang's vertical side walls (x = 0 and x = 10 are flat
// faces of the extruded profile), at least 0.5 mm clear of any edge. Their
// normals are horizontal, polar = 90 deg, so Phase 3 would drop them at any
// setting above zero if it ever saw them.
sla::SupportPoints carried_wall_points()
{
    sla::SupportPoints pts;
    for (const Vec3f &p : {Vec3f(10.f, 0.5f, 5.f), Vec3f(0.f, 0.5f, 5.f)}) {
        sla::SupportPoint sp;
        sp.pos               = p;
        sp.head_front_radius = 0.2f;
        sp.type              = sla::SupportPointType::manual_add;
        pts.push_back(sp);
    }
    return pts;
}

} // namespace

TEST_CASE("Points the caller carried in survive the angle filter",
          "[SLAOverhangFilter][SLASupportGeneration]")
{
    // capability sla-overhang-threshold-semantics, scenario
    // "使用者攜帶的點不受第 5 步過濾影響".
    const sla::SupportPoints carried = carried_wall_points();

    const size_t plain_45   = support_point_count_with_carried(45., {});
    const size_t carried_45 = support_point_count_with_carried(45., carried);
    const size_t plain_90   = support_point_count_with_carried(90., {});
    const size_t carried_90 = support_point_count_with_carried(90., carried);

    INFO("45 deg: " << plain_45 << " -> " << carried_45
                    << "   90 deg: " << plain_90 << " -> " << carried_90);

    // The automatic half still moves with the setting, or the comparison below
    // would be between two runs that were never filtered at all.
    REQUIRE(plain_90 < plain_45);

    // Checked before subtracting, not for tidiness: these are size_t, so a run
    // where the carried points somehow REMOVED output - deduplicated against a
    // generated point, say - would underflow into a huge number and sail
    // through both assertions below.
    REQUIRE(carried_45 >= plain_45);
    REQUIRE(carried_90 >= plain_90);

    const size_t extra_45 = carried_45 - plain_45;
    const size_t extra_90 = carried_90 - plain_90;

    // At least one carried point actually reached the output. A run where
    // prepare_permanent_support_points() dropped them all - for being off the
    // surface, say - would satisfy the equality below while proving nothing.
    REQUIRE(extra_45 > 0);

    // The point of the whole case: the number of carried points that come
    // through does not depend on the angle, even though these sit on vertical
    // walls that the filter rejects at 90 degrees.
    REQUIRE(extra_90 == extra_45);
}

TEST_CASE("An imported list replaces automatic detection outright",
          "[SLAOverhangFilter][SLASupportGeneration]")
{
    // capability support-point-interchange, scenarios "傳入的點被完整採用"
    // and "不執行自動偵測", stated against each other so the contrast is the
    // assertion rather than a comment.
    Model model = FileReader::load_model(TEST_DATA_DIR PATH_SEPARATOR
                                         "U_overhang.obj");
    REQUIRE_FALSE(model.objects.empty());

    // Every size field carries a distinctive value, so "taken completely" can
    // be checked field by field rather than inferred from the count. Leaving
    // them at the sentinel would make the loop at the end pass on a step that
    // had quietly overwritten each one with the global default.
    sla::SupportPoints imported;
    for (const Vec3f &p : {Vec3f(5.f, 5.f, 0.f), Vec3f(0.f, 0.5f, 5.f),
                           Vec3f(10.f, 0.5f, 5.f)}) {
        sla::SupportPoint sp;
        sp.pos                       = p;
        sp.type                      = sla::SupportPointType::manual_add;
        sp.head_front_radius         = 0.2f;
        sp.head_back_radius_mm       = 0.6f;
        sp.head_width_mm             = 1.2f;
        sp.head_penetration_mm       = 0.35f;
        sp.contact_sphere_radius     = 0.f; // explicit "off", not the sentinel
        sp.base_radius_mm            = 2.5f;
        sp.support_bracing_angle_deg = 40.f;
        imported.push_back(sp);
    }

    ModelObject *mo        = model.objects.front();
    mo->sla_support_points = imported;
    mo->sla_points_status  = sla::PointsStatus::UserModified;

    SLAFullPrintConfig fullcfg;
    fullcfg.printer_technology.setInt(ptSLA);
    fullcfg.supports_enable.value        = true;
    fullcfg.pad_enable.value             = false;
    fullcfg.support_critical_angle.value = 45.;
    fullcfg.set("layer_height", DETECTION_LAYER_HEIGHT);
    fullcfg.set("initial_layer_height", DETECTION_LAYER_HEIGHT);

    DynamicPrintConfig cfg;
    cfg.apply(fullcfg);

    SLAPrint print;
    print.set_status_callback([](const PrintBase::SlicingStatus &) {});
    print.apply(model, cfg);

    PrintBase::TaskParams task;
    task.to_object_step = slaposSupportPoints;
    print.set_task(task);
    print.process();

    REQUIRE_FALSE(print.objects().empty());
    const sla::SupportPoints &out = print.objects().front()->get_support_points();

    // What automatic detection produces for this model at this setting. The
    // imported run must not look anything like it.
    const size_t automatic = support_point_count("U_overhang", 45.);
    INFO("imported " << out.size() << " vs automatic " << automatic);

    REQUIRE(automatic == 20u);
    REQUIRE(out.size() == imported.size()); // every point taken, none added
    REQUIRE(out.size() != automatic);       // detection genuinely did not run

    // Positions go through the object transform, but nothing else about a
    // point may be rewritten on the way through - all seven size fields and
    // the type come back exactly as they were handed in.
    for (const sla::SupportPoint &sp : out) {
        REQUIRE(sp.type                      == sla::SupportPointType::manual_add);
        REQUIRE(sp.head_front_radius         == 0.2f);
        REQUIRE(sp.head_back_radius_mm       == 0.6f);
        REQUIRE(sp.head_width_mm             == 1.2f);
        REQUIRE(sp.head_penetration_mm       == 0.35f);
        REQUIRE(sp.contact_sphere_radius     == 0.f);
        REQUIRE(sp.base_radius_mm            == 2.5f);
        REQUIRE(sp.support_bracing_angle_deg == 40.f);
    }
}

// --- Task 5.3: nothing in the exported list is headless for want of angle ---
//
// The promise this whole change exists to keep. A point may still fail to grow
// a head - to a collision, to the 0.1 mm cluster dedup, to a routing search
// that found nowhere to go - but it must never fail for an overhang angle,
// because the angle was already decided upstream in Phase 3.
//
// Step 6's own emesh and config are rebuilt here from the print object's public
// accessors, so the tree sees the same mesh and the same head_front_radius_mm
// it would see inside the pipeline. SupportableMesh::zoffset is left at 0
// rather than the csgmesh bounding box the real step 6 computes - it moves the
// ground plane, so WHICH points end up headless can differ from a production
// run. That does not weaken the assertion: whatever this tree left headless is
// exactly the set being checked.
//
// Read the INFO line before trusting a pass. If headless comes out 0 the loop
// body never runs and the case is vacuously true - still an honest result (the
// tree dropped nobody), but it proves nothing about the angle in particular.
// It is deliberately not asserted non-zero: the count depends on the zoffset
// this case cannot reproduce, so pinning a number here would be a guess.

TEST_CASE("No point in the exported list is headless for want of angle",
          "[SLAOverhangFilter][SLASupportGeneration]")
{
    Model model = FileReader::load_model(TEST_DATA_DIR PATH_SEPARATOR
                                         "frog_legs.obj");

    SLAFullPrintConfig fullcfg;
    fullcfg.printer_technology.setInt(ptSLA);
    fullcfg.supports_enable.value        = true;
    fullcfg.pad_enable.value             = false;
    fullcfg.support_critical_angle.value = 45.;
    fullcfg.set("layer_height", DETECTION_LAYER_HEIGHT);
    fullcfg.set("initial_layer_height", DETECTION_LAYER_HEIGHT);

    DynamicPrintConfig cfg;
    cfg.apply(fullcfg);

    SLAPrint print;
    print.set_status_callback([](const PrintBase::SlicingStatus &) {});
    print.apply(model, cfg);

    PrintBase::TaskParams task;
    task.to_object_step = slaposSupportPoints;
    print.set_task(task);
    print.process();

    REQUIRE_FALSE(print.objects().empty());
    const SLAPrintObject     *po  = print.objects().front();
    const sla::SupportPoints  pts = po->get_support_points();

    // frog_legs is the fixture here rather than U_overhang because its organic
    // surface is where the angle filter actually bites - 172 sampled points,
    // 167 surviving. An empty list would make everything below vacuous.
    REQUIRE(pts.size() > 0);

    const std::shared_ptr<const indexed_triangle_set> &meshp = po->get_mesh_to_print();
    REQUIRE(meshp);

    const sla::SupportTreeConfig scfg = make_support_cfg(po->config());

    sla::SupportableMesh    sm{*meshp, pts, scfg};
    sla::SupportTreeBuilder builder;
    sla::DefaultSupportTree::execute(builder, sm);

    // The same normals the tree itself computed: same mesh, same eps.
    PointSet point_matrix(pts.size(), 3);
    for (size_t i = 0; i < pts.size(); ++i)
        point_matrix.row(Eigen::Index(i)) = pts[i].pos.cast<double>();

    const PointSet nmls = Slic3r::normals(ex_tbb, point_matrix, sm.emesh,
                                          scfg.head_front_radius_mm);

    size_t headless = 0;
    size_t headless_rejected_by_angle = 0;

    for (size_t i = 0; i < pts.size(); ++i) {
        if (head_of(builder, long(i)) != nullptr)
            continue;

        ++headless;

        const Vec3d  n     = nmls.row(Eigen::Index(i));
        const double polar = Geometry::dir_to_spheric(n).first;
        if (!passes_overhang_filter(polar, scfg.overhang_angle_threshold))
            ++headless_rejected_by_angle;
    }

    INFO("points=" << pts.size() << " headless=" << headless
                   << " of which rejected by angle=" << headless_rejected_by_angle);

    // The claim, stated directly: of the points that grew no head, not one of
    // them would have been turned away by the overhang filter. Every survivor
    // of Phase 3 is angle-clean by construction, so a non-zero count here would
    // mean the two stages had drifted apart.
    REQUIRE(headless_rejected_by_angle == 0);
}

// --- Task 5.4: export, re-import, re-export ---------------------------------
//
// Three properties, because they fail in different ways:
//   1. the serializer preserves a real engine point list
//   2. the round trip settles - points immediately, text one step later
//   3. the engine consumes the parsed list without rewriting anything
//
// Both notes below come from the same fact: the exporter FREEZES a point,
// resolving every unset size field against the config rather than writing a
// sentinel (sla_support_point_io_tests.cpp, "An unset point is exported
// frozen"), and a size on the CONFIG is a double while the same size on a
// SupportPoint is a float.
//
// Property 1: auto generated points carry sentinels, so their size fields
// legitimately differ between the engine list and the parsed one. Position and
// type are never resolved, and those are what property 1 compares.
//
// What this case does NOT cover: the CLI exporter maps every position back
// through the inverse object transform before writing the file
// (CLI/ProcessActions.cpp, around the model_fingerprint call). This case
// serialises po->get_support_points() as it stands, so it pins the serializer
// and the engine's copy semantics, not the CLI's coordinate handling. A defect
// in that mapping would slip past here.
//
// Property 2: the first export writes a fallen-back size at DOUBLE precision
// (0.2), parsing narrows it to a float, and every export after that writes it
// at FLOAT precision (0.20000000298023224). So text1 != text2 by design - the
// text settles from the SECOND export onwards, while the points settle from the
// first import. This is spelled out in sla_support_point_io_tests.cpp,
// "Repeated round trips reach a fixed point"; asserting text2 == text1 here
// contradicts it.

TEST_CASE("An exported point list survives being imported and exported again",
          "[SLAOverhangFilter][SLASupportGeneration]")
{
    // ---- generate a real list -------------------------------------------
    Model model = FileReader::load_model(TEST_DATA_DIR PATH_SEPARATOR
                                         "U_overhang.obj");

    SLAFullPrintConfig fullcfg;
    fullcfg.printer_technology.setInt(ptSLA);
    fullcfg.supports_enable.value        = true;
    fullcfg.pad_enable.value             = false;
    fullcfg.support_critical_angle.value = 45.;
    fullcfg.set("layer_height", DETECTION_LAYER_HEIGHT);
    fullcfg.set("initial_layer_height", DETECTION_LAYER_HEIGHT);

    DynamicPrintConfig cfg;
    cfg.apply(fullcfg);

    SLAPrint print;
    print.set_status_callback([](const PrintBase::SlicingStatus &) {});
    print.apply(model, cfg);

    PrintBase::TaskParams task;
    task.to_object_step = slaposSupportPoints;
    print.set_task(task);
    print.process();

    REQUIRE_FALSE(print.objects().empty());
    const SLAPrintObject     *po       = print.objects().front();
    const sla::SupportPoints  exported = po->get_support_points();
    REQUIRE(exported.size() == 20u);

    const sla::SupportTreeConfig scfg = make_support_cfg(po->config());
    const sla::ModelFingerprint  fp   = sla::model_fingerprint(*po->model_object());

    // ---- property 1: the serializer keeps what it was given ---------------
    const std::string text1 = sla::support_points_to_string(exported, fp, scfg);

    sla::SupportPointFile parsed;
    std::string           err;
    // INFO streams into the message the moment it is constructed, so err has
    // to be filled in BEFORE it, not by the REQUIRE on the same line.
    const bool parsed_ok = sla::support_points_from_string(text1, scfg, parsed, err);
    INFO("parse error: " << err);
    REQUIRE(parsed_ok);
    REQUIRE(parsed.points.size() == exported.size());

    for (size_t i = 0; i < exported.size(); ++i) {
        INFO("point " << i);
        REQUIRE(parsed.points[i].pos  == exported[i].pos);
        REQUIRE(parsed.points[i].type == exported[i].type);
    }

    // ---- property 2: the round trip settles --------------------------------
    REQUIRE(parsed.has_fingerprint);
    const std::string text2 =
        sla::support_points_to_string(parsed.points, parsed.fingerprint, scfg);

    sla::SupportPointFile parsed2;
    const bool parsed2_ok = sla::support_points_from_string(text2, scfg, parsed2, err);
    INFO("second parse error: " << err);
    REQUIRE(parsed2_ok);

    const std::string text3 =
        sla::support_points_to_string(parsed2.points, parsed2.fingerprint, scfg);

    // The text is a fixed point from the second export onwards.
    REQUIRE(text3 == text2);

    // The points are a fixed point from the first import onwards - bit for bit,
    // not the tolerant operator==.
    REQUIRE(parsed2.points.size() == parsed.points.size());
    for (size_t i = 0; i < parsed.points.size(); ++i) {
        INFO("point " << i);
        REQUIRE(parsed2.points[i].pos                      == parsed.points[i].pos);
        REQUIRE(parsed2.points[i].type                     == parsed.points[i].type);
        REQUIRE(parsed2.points[i].head_front_radius        == parsed.points[i].head_front_radius);
        REQUIRE(parsed2.points[i].head_back_radius_mm      == parsed.points[i].head_back_radius_mm);
        REQUIRE(parsed2.points[i].head_width_mm            == parsed.points[i].head_width_mm);
        REQUIRE(parsed2.points[i].head_penetration_mm      == parsed.points[i].head_penetration_mm);
        REQUIRE(parsed2.points[i].contact_sphere_radius    == parsed.points[i].contact_sphere_radius);
        REQUIRE(parsed2.points[i].base_radius_mm           == parsed.points[i].base_radius_mm);
        REQUIRE(parsed2.points[i].support_bracing_angle_deg
                == parsed.points[i].support_bracing_angle_deg);
    }

    // ---- property 3: the engine takes the parsed list verbatim -------------
    Model model2 = FileReader::load_model(TEST_DATA_DIR PATH_SEPARATOR
                                          "U_overhang.obj");
    REQUIRE_FALSE(model2.objects.empty());
    model2.objects.front()->sla_support_points = parsed.points;
    model2.objects.front()->sla_points_status  = sla::PointsStatus::UserModified;

    DynamicPrintConfig cfg2;
    cfg2.apply(fullcfg);

    SLAPrint print2;
    print2.set_status_callback([](const PrintBase::SlicingStatus &) {});
    print2.apply(model2, cfg2);
    print2.set_task(task);
    print2.process();

    REQUIRE_FALSE(print2.objects().empty());
    const SLAPrintObject     *po2       = print2.objects().front();
    const sla::SupportPoints &reimported = po2->get_support_points();

    REQUIRE(reimported.size() == parsed.points.size());

    // transformed_support_points() writes pos and copies everything else
    // verbatim (SLAPrint.cpp), so the expected position is the same product it
    // computes - not an approximation of it.
    const Transform3f tr = po2->trafo().cast<float>();

    for (size_t i = 0; i < reimported.size(); ++i) {
        INFO("point " << i);
        REQUIRE(reimported[i].pos == (tr * parsed.points[i].pos));
        REQUIRE(reimported[i].type                      == parsed.points[i].type);
        REQUIRE(reimported[i].head_front_radius         == parsed.points[i].head_front_radius);
        REQUIRE(reimported[i].head_back_radius_mm       == parsed.points[i].head_back_radius_mm);
        REQUIRE(reimported[i].head_width_mm             == parsed.points[i].head_width_mm);
        REQUIRE(reimported[i].head_penetration_mm       == parsed.points[i].head_penetration_mm);
        REQUIRE(reimported[i].contact_sphere_radius     == parsed.points[i].contact_sphere_radius);
        REQUIRE(reimported[i].base_radius_mm            == parsed.points[i].base_radius_mm);
        REQUIRE(reimported[i].support_bracing_angle_deg == parsed.points[i].support_bracing_angle_deg);
    }
}

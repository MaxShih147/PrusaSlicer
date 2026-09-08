#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <optional>

#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/SLA/DefaultSupportTree.hpp"
#include "libslic3r/SLA/SupportTree.hpp"
#include "libslic3r/SLA/SupportTreeBuilder.hpp"
#include "libslic3r/SLA/SupportTreeUtils.hpp"
#include "libslic3r/SLA/SupportTreeUtilsLegacy.hpp"
#include "libslic3r/Execution/ExecutionSeq.hpp"

using namespace Slic3r;

// Per-point support geometry actually reaching the built tree (openspec change
// per-point-support-sizing, tasks 3.1 - 3.4). The unit tests in
// sla_support_point_fields_tests.cpp cover the resolvers in isolation; these
// build a real support tree and read the sizes back off the structures, which
// is the only way to catch a resolver that is correct but wired to nothing.

namespace {

constexpr double SizeTol  = 1e-6;
constexpr double AngleTol = 1e-3; // rad

// A flat-bottomed disk. The whole underside is a horizontal overhang, so every
// support point placed on it gets a head pointing straight down and the
// bracing angle never binds - which is what the pillar-radius tests want.
indexed_triangle_set flat_bottom_disk(double r, double h)
{
    return its_make_cylinder(r, h);
}

// A cone standing on its apex. its_make_cone() puts the apex at the top, so
// mirror it in Z and flip the winding to keep the normals outward. The result
// has a lateral surface that faces down and outward at a shallow angle, which
// is what makes the bracing-angle saturation observable.
indexed_triangle_set apex_down_cone(double r, double h)
{
    indexed_triangle_set c = its_make_cone(r, h);
    for (Vec3f &v : c.vertices)
        v.z() = float(h) - v.z();
    for (Vec3i32 &f : c.indices)
        std::swap(f(1), f(2));
    return c;
}

sla::SupportTreeConfig test_cfg()
{
    // Plain defaults. This used to force overhang_angle_threshold to 0 so the
    // cone cases would not lose their heads to the tree's own angle gate; that
    // gate is gone - filtering happens once, upstream, in Phase 3 of
    // support_points() - so the field is no longer read here and setting it
    // did nothing (openspec change align-support-point-overhang-filter,
    // capability sla-overhang-threshold-semantics).
    return sla::SupportTreeConfig{};
}

// The head grown from support point `pt_index`, or nullptr if the point was
// filtered out. Head IDs are support point indices by construction.
const sla::Head *head_of(const sla::SupportTreeBuilder &b, long pt_index)
{
    auto it = std::find_if(b.heads().begin(), b.heads().end(),
                           [pt_index](const sla::Head &h) {
                               return h.is_valid() && h.id == pt_index;
                           });
    return it == b.heads().end() ? nullptr : &*it;
}

double head_polar(const sla::Head &h)
{
    return std::get<0>(sla::dir_to_spheric(h.dir));
}

} // namespace

TEST_CASE("An instance scale moves the point but never its sizes",
          "[sla_per_point_geometry]")
{
    // The seven size fields are absolute millimetres. Scaling the object must
    // move WHERE a support attaches and nothing else - a 2x model does not get
    // 2x thick pillars.
    //
    // This is exactly what SLAPrintObject::transformed_support_points() does
    // before handing the list to the tree: it writes suppt.pos and leaves every
    // other field alone. Reproduced here rather than driving a whole SLAPrint,
    // because the property under test is a property of that one assignment.
    sla::SupportPoint sp;
    sp.pos                       = Vec3f(3.f, 4.f, 5.f);
    sp.type                      = sla::SupportPointType::slope;
    sp.head_front_radius         = 0.4f;
    sp.head_back_radius_mm       = 0.6f;
    sp.head_width_mm             = 1.2f;
    sp.head_penetration_mm       = 0.35f;
    sp.contact_sphere_radius     = 0.f;   // explicit "off", not the sentinel
    sp.base_radius_mm            = 2.5f;
    sp.support_bracing_angle_deg = 40.f;

    const sla::SupportPoint before = sp;

    Transform3d scl = Transform3d::Identity();
    scl.scale(2.);
    const Transform3f tr = scl.cast<float>();
    sp.pos = tr * sp.pos;

    // The position did scale - otherwise this test proves nothing.
    REQUIRE(std::abs(sp.pos.x() - 6.f) < SizeTol);
    REQUIRE(std::abs(sp.pos.y() - 8.f) < SizeTol);
    REQUIRE(std::abs(sp.pos.z() - 10.f) < SizeTol);

    // ... and not one of the seven sizes moved with it.
    REQUIRE(sp.head_front_radius         == before.head_front_radius);
    REQUIRE(sp.head_back_radius_mm       == before.head_back_radius_mm);
    REQUIRE(sp.head_width_mm             == before.head_width_mm);
    REQUIRE(sp.head_penetration_mm       == before.head_penetration_mm);
    REQUIRE(sp.contact_sphere_radius     == before.contact_sphere_radius);
    REQUIRE(sp.base_radius_mm            == before.base_radius_mm);
    REQUIRE(sp.support_bracing_angle_deg == before.support_bracing_angle_deg);
}

TEST_CASE("A doubled model builds the same pillar radius as the original",
          "[sla_per_point_geometry]")
{
    // The end-to-end half of the property above: the same 0.6 mm request on a
    // model twice the size must still come out 0.6 mm on the built head. A
    // resolver that scaled with the geometry would read 1.2 here.
    sla::SupportTreeConfig cfg = test_cfg();

    auto build_at = [&](double scale) {
        indexed_triangle_set disk = flat_bottom_disk(8. * scale, 5. * scale);

        sla::SupportPoint sp;
        sp.pos                 = Vec3f(0.f, 0.f, 0.f); // flat underside, either size
        sp.head_front_radius   = float(cfg.head_front_radius_mm);
        sp.type                = sla::SupportPointType::slope;
        sp.head_back_radius_mm = 0.6f; // absolute mm, NOT scaled with the model
        sp.base_radius_mm      = 2.5f;

        return std::make_pair(std::move(disk), sla::SupportPoints{sp});
    };

    auto one = build_at(1.);
    auto two = build_at(2.);

    sla::SupportableMesh sm1{one.first, one.second, cfg};
    sla::SupportTreeBuilder b1;
    sla::DefaultSupportTree::execute(b1, sm1);

    sla::SupportableMesh sm2{two.first, two.second, cfg};
    sla::SupportTreeBuilder b2;
    sla::DefaultSupportTree::execute(b2, sm2);

    const sla::Head *h1 = head_of(b1, 0);
    const sla::Head *h2 = head_of(b2, 0);
    REQUIRE(h1);
    REQUIRE(h2);

    REQUIRE(std::abs(h1->r_back_mm - 0.6) < SizeTol);
    REQUIRE(std::abs(h2->r_back_mm - 0.6) < SizeTol);
    REQUIRE(std::abs(h1->r_back_mm - h2->r_back_mm) < SizeTol);
}

TEST_CASE("A custom pillar radius on one point leaves its neighbours alone",
          "[sla_per_point_geometry]")
{
    sla::SupportTreeConfig cfg = test_cfg();

    indexed_triangle_set disk = flat_bottom_disk(8., 5.);
    const double zbottom = 0.;

    sla::SupportPoints pts;
    for (double x : {-3., 0., 3.}) {
        sla::SupportPoint sp;
        sp.pos               = Vec3f(float(x), 0.f, float(zbottom));
        sp.head_front_radius = float(cfg.head_front_radius_mm);
        sp.type              = sla::SupportPointType::slope;
        pts.push_back(sp);
    }
    // Only the middle point is thickened.
    pts[1].head_back_radius_mm = 0.6f;

    sla::SupportableMesh sm{disk, pts, cfg};
    sla::SupportTreeBuilder builder;
    sla::DefaultSupportTree::execute(builder, sm);

    const sla::Head *h0 = head_of(builder, 0);
    const sla::Head *h1 = head_of(builder, 1);
    const sla::Head *h2 = head_of(builder, 2);
    REQUIRE(h0);
    REQUIRE(h1);
    REQUIRE(h2);

    REQUIRE(std::abs(h1->r_back_mm - 0.6) < SizeTol);
    REQUIRE(std::abs(h0->r_back_mm - cfg.head_back_radius_mm) < SizeTol);
    REQUIRE(std::abs(h2->r_back_mm - cfg.head_back_radius_mm) < SizeTol);
}

TEST_CASE("Three points each keep their own pillar radius",
          "[sla_per_point_geometry]")
{
    sla::SupportTreeConfig cfg = test_cfg();

    indexed_triangle_set disk = flat_bottom_disk(8., 5.);
    const double radii[3] = {0.3, 0.6, 0.9};

    sla::SupportPoints pts;
    for (int i = 0; i < 3; ++i) {
        sla::SupportPoint sp;
        sp.pos                 = Vec3f(float(-3 + 3 * i), 0.f, 0.f);
        sp.head_front_radius   = float(cfg.head_front_radius_mm);
        sp.type                = sla::SupportPointType::slope;
        sp.head_back_radius_mm = float(radii[i]);
        pts.push_back(sp);
    }

    sla::SupportableMesh sm{disk, pts, cfg};
    sla::SupportTreeBuilder builder;
    sla::DefaultSupportTree::execute(builder, sm);

    for (int i = 0; i < 3; ++i) {
        const sla::Head *h = head_of(builder, i);
        REQUIRE(h);
        REQUIRE(std::abs(h->r_back_mm - radii[i]) < SizeTol);
    }
}

TEST_CASE("A custom head width and penetration reach the head",
          "[sla_per_point_geometry]")
{
    sla::SupportTreeConfig cfg = test_cfg();

    indexed_triangle_set disk = flat_bottom_disk(8., 5.);

    sla::SupportPoint sp;
    sp.pos                 = Vec3f(0.f, 0.f, 0.f);
    sp.head_front_radius   = float(cfg.head_front_radius_mm);
    sp.type                = sla::SupportPointType::slope;
    sp.head_width_mm       = 2.5f;
    sp.head_penetration_mm = 0.1f;

    sla::SupportableMesh sm{disk, {sp}, cfg};
    sla::SupportTreeBuilder builder;
    sla::DefaultSupportTree::execute(builder, sm);

    const sla::Head *h = head_of(builder, 0);
    REQUIRE(h);
    REQUIRE(std::abs(h->width_mm - 2.5) < SizeTol);
    // The committed penetration is clamped against the local wall thickness,
    // so it can only be at most what the point asked for - never the (larger)
    // global default.
    REQUIRE(h->penetration_mm <= 0.1 + SizeTol);
    REQUIRE(h->penetration_mm < cfg.head_penetration_mm);
}

TEST_CASE("An island point's base radius reaches the pillar base",
          "[sla_per_point_geometry]")
{
    sla::SupportTreeConfig cfg = test_cfg();

    indexed_triangle_set disk = flat_bottom_disk(8., 5.);

    auto build = [&](float base_radius) {
        sla::SupportPoint sp;
        sp.pos               = Vec3f(0.f, 0.f, 0.f);
        sp.head_front_radius = float(cfg.head_front_radius_mm);
        // Autogenerated island point, not manual_add: the size still applies.
        sp.type              = sla::SupportPointType::island;
        sp.base_radius_mm    = base_radius;
        return sp;
    };

    double base_default = 0., base_custom = 0.;

    {
        sla::SupportableMesh sm{disk, {build(sla::support_point_unset)}, cfg};
        sla::SupportTreeBuilder builder;
        sla::DefaultSupportTree::execute(builder, sm);
        REQUIRE_FALSE(builder.pedestals().empty());
        base_default = builder.pedestals().front().r_bottom;
    }

    {
        sla::SupportableMesh sm{disk, {build(3.f)}, cfg};
        sla::SupportTreeBuilder builder;
        sla::DefaultSupportTree::execute(builder, sm);
        REQUIRE_FALSE(builder.pedestals().empty());
        base_custom = builder.pedestals().front().r_bottom;
    }

    REQUIRE(std::abs(base_default - cfg.base_radius_mm) < SizeTol);
    REQUIRE(std::abs(base_custom - 3.0) < SizeTol);
}

TEST_CASE("A slope point's bracing angle reaches the head direction",
          "[sla_per_point_geometry]")
{
    sla::SupportTreeConfig cfg = test_cfg();

    // Base radius 5, height 10: the lateral normal sits at ~117 degrees polar,
    // shallower than either bracing bound below, so the saturation is what
    // decides the head direction in both runs.
    indexed_triangle_set cone = apex_down_cone(5., 10.);

    auto build = [&](float angle_deg) {
        sla::SupportPoint sp;
        // Halfway up the slant: at height h/2 the cone radius is r/2.
        sp.pos                       = Vec3f(2.5f, 0.f, 5.f);
        sp.head_front_radius         = float(cfg.head_front_radius_mm);
        // Autogenerated slope point, not manual_add: the angle still applies.
        sp.type                      = sla::SupportPointType::slope;
        sp.support_bracing_angle_deg = angle_deg;
        return sp;
    };

    double polar_default = 0., polar_custom = 0.;

    {
        sla::SupportableMesh sm{cone, {build(sla::support_point_unset)}, cfg};
        sla::SupportTreeBuilder builder;
        sla::DefaultSupportTree::execute(builder, sm);
        const sla::Head *h = head_of(builder, 0);
        REQUIRE(h);
        polar_default = head_polar(*h);
    }

    {
        sla::SupportableMesh sm{cone, {build(20.f)}, cfg};
        sla::SupportTreeBuilder builder;
        sla::DefaultSupportTree::execute(builder, sm);
        const sla::Head *h = head_of(builder, 0);
        REQUIRE(h);
        polar_custom = head_polar(*h);
    }

    // Unset falls back to the global bridge_slope...
    REQUIRE(std::abs(polar_default - (PI - cfg.bridge_slope)) < AngleTol);
    // ...and 20 degrees pushes the head that much closer to vertical.
    REQUIRE(std::abs(polar_custom - (PI - 20. * sla::support_point_deg_to_rad)) < AngleTol);
    // Guard against a vacuous test: the two runs must actually differ.
    REQUIRE(std::abs(polar_custom - polar_default) > AngleTol);
}

TEST_CASE("All-sentinel points reproduce the global configuration exactly",
          "[sla_per_point_geometry]")
{
    sla::SupportTreeConfig cfg = test_cfg();

    indexed_triangle_set disk = flat_bottom_disk(8., 5.);

    sla::SupportPoints pts;
    for (double x : {-3., 0., 3.}) {
        sla::SupportPoint sp;
        sp.pos               = Vec3f(float(x), 0.f, 0.f);
        sp.head_front_radius = float(cfg.head_front_radius_mm);
        sp.type              = sla::SupportPointType::slope;
        pts.push_back(sp); // every extension field left at the sentinel
    }

    sla::SupportableMesh sm{disk, pts, cfg};
    sla::SupportTreeBuilder builder;
    sla::DefaultSupportTree::execute(builder, sm);

    REQUIRE(builder.heads().size() == 3);
    for (const sla::Head &h : builder.heads()) {
        REQUIRE(std::abs(h.r_back_mm - cfg.head_back_radius_mm) < SizeTol);
        REQUIRE(std::abs(h.width_mm - cfg.head_width_mm) < SizeTol);
        REQUIRE(h.penetration_mm <= cfg.head_penetration_mm + SizeTol);
    }
    for (const sla::Pedestal &p : builder.pedestals())
        REQUIRE(p.r_bottom >= cfg.base_radius_mm - SizeTol);
}

TEST_CASE("The widening search ends at the radius it was asked for",
          "[sla_per_point_geometry]")
{
    // create_ground_pillar() widens a mini pillar up to the radius its own
    // support point asked for, then tests `radius >= that radius` to decide
    // whether a pillar base fits. Returning the global default here made that
    // test fail for every point wider than the default: no pedestal, and the
    // ground level dropped by a pad wall thickness.
    sla::SupportTreeConfig cfg = test_cfg();

    indexed_triangle_set disk = flat_bottom_disk(4., 1.);
    sla::SupportableMesh sm{disk, sla::SupportPoints{}, cfg};

    const double from_r = 0.25; // the head_fallback_radius_mm mini pillar
    // Off to the side of the disk so the route straight down is clear and the
    // search actually succeeds.
    const Vec3d jp{10., 0., 10.};

    for (double target : {0.5, 0.9}) {
        std::optional<sla::DiffBridge> br =
            sla::search_widening_path(ex_seq, sm, jp, sla::DOWN, from_r, target);
        REQUIRE(br);
        REQUIRE(std::abs(br->end_r - target) < SizeTol);
        REQUIRE(std::abs(br->r - from_r) < SizeTol);
    }
}

TEST_CASE("Zero elevation with a steep bracing angle stays finite",
          "[sla_per_point_geometry]")
{
    // The zero-elevation corrector bridge divides by sqrt(1 - slope^2) with
    // slope in radians, so 1 rad (~57.3 deg) and above used to produce a NaN
    // that std::min() passed through as "no ground clearance limit".
    sla::SupportTreeConfig cfg = test_cfg();
    cfg.object_elevation_mm = 0.;

    indexed_triangle_set disk = flat_bottom_disk(8., 5.);

    for (float angle_deg : {45.f, 80.f}) {
        sla::SupportPoint sp;
        sp.pos                       = Vec3f(0.f, 0.f, 0.f);
        sp.head_front_radius         = float(cfg.head_front_radius_mm);
        sp.type                      = sla::SupportPointType::slope;
        sp.support_bracing_angle_deg = angle_deg;

        sla::SupportableMesh sm{disk, {sp}, cfg};
        sla::SupportTreeBuilder builder;
        // Must terminate, and must not leave a NaN anywhere in the geometry.
        sla::DefaultSupportTree::execute(builder, sm);

        for (const sla::Head &h : builder.heads()) {
            REQUIRE(h.dir.allFinite());
            REQUIRE(h.pos.allFinite());
            REQUIRE(std::isfinite(h.width_mm));
            REQUIRE(std::isfinite(h.penetration_mm));
        }
        for (const sla::Pillar &p : builder.pillars()) {
            REQUIRE(std::isfinite(p.height));
            REQUIRE(std::isfinite(p.r_start));
            REQUIRE(p.endpt.allFinite());
        }
        for (const sla::Pedestal &p : builder.pedestals()) {
            REQUIRE(std::isfinite(p.r_bottom));
            REQUIRE(p.pos.allFinite());
        }
    }
}

TEST_CASE("Pillar clustering measures both points' own base radii",
          "[sla_per_point_geometry]")
{
    // The clustering predicate asks "would these two pillar bases overlap".
    // Two points 5 mm apart clear a pair of 2 mm default bases (sum 4) but not
    // a pair of custom 3 mm ones (sum 6), so widening the bases has to pull the
    // two pillars into one cluster.
    sla::SupportTreeConfig cfg = test_cfg();

    indexed_triangle_set disk = flat_bottom_disk(10., 5.);

    auto count_pedestals = [&](float base_radius) {
        sla::SupportPoints pts;
        for (double x : {-2.5, 2.5}) {
            sla::SupportPoint sp;
            sp.pos               = Vec3f(float(x), 0.f, 0.f);
            sp.head_front_radius = float(cfg.head_front_radius_mm);
            sp.type              = sla::SupportPointType::island;
            sp.base_radius_mm    = base_radius;
            pts.push_back(sp);
        }
        sla::SupportableMesh sm{disk, pts, cfg};
        sla::SupportTreeBuilder builder;
        sla::DefaultSupportTree::execute(builder, sm);
        return builder.pedestals().size();
    };

    const size_t n_default = count_pedestals(sla::support_point_unset);
    const size_t n_wide    = count_pedestals(3.f);

    // Guard against a vacuous test: the default pair must really stay apart.
    REQUIRE(n_default == 2);
    REQUIRE(n_wide < n_default);
}

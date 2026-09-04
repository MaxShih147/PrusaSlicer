///|/ Copyright (c) Prusa Research 2020 - 2023 Vojtěch Bubník @bubnikv, Tomáš Mészáros @tamasmeszaros
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef SLA_SUPPORTTREE_HPP
#define SLA_SUPPORTTREE_HPP

#include <libslic3r/Polygon.hpp>
#include <libslic3r/ExPolygon.hpp>
#include <libslic3r/AABBMesh.hpp>
#include <libslic3r/SLA/Pad.hpp>
#include <libslic3r/SLA/SupportPoint.hpp>
#include <libslic3r/SLA/JobController.hpp>
#include <libslic3r/SLA/SupportTreeStrategies.hpp>
#include <math.h>
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>

#include "admesh/stl.h"

namespace Slic3r {

namespace sla {
struct JobController;

struct SupportTreeConfig
{
    bool   enabled = true;

    // Type of the support tree, for
    SupportTreeType tree_type = SupportTreeType::Default;

    // Radius in mm of the pointing side of the head.
    double head_front_radius_mm = 0.2;

    // How much the pinhead has to penetrate the model surface
    double head_penetration_mm = 0.5;

    // Radius of the back side of the 3d arrow.
    double head_back_radius_mm = 0.5;

    double head_fallback_radius_mm = 0.25;

    // Width in mm from the back sphere center to the front sphere center.
    double head_width_mm = 1.0;

    // How to connect pillars
    PillarConnectionMode pillar_connection_mode = PillarConnectionMode::dynamic;

    // Only generate pillars that can be routed to ground
    bool ground_facing_only = false;

    // TODO: unimplemented at the moment. This coefficient will have an impact
    // when bridges and pillars are merged. The resulting pillar should be a bit
    // thicker than the ones merging into it. How much thicker? I don't know
    // but it will be derived from this value.
    double pillar_widening_factor = .5;

    // Radius in mm of the pillar base.
    double base_radius_mm = 2.0;

    // The height of the pillar base cone in mm.
    double base_height_mm = 1.0;

    // The default angle for connecting support sticks and junctions.
    double bridge_slope = M_PI/4;

    // Overhang angle threshold in radians. A support head is placed only where
    // the surface's slope from the horizontal plane is at most
    // (PI/2 - overhang_angle_threshold), so a SMALLER value supports MORE
    // surfaces: 0 supports every overhang, PI/2 (the default) supports only
    // perfectly horizontal down-facing surfaces.
    //
    // CONSUMED IN EXACTLY ONE PLACE: the Phase 3 filter in
    // SLAPrintSteps.cpp's support_points() (step slaposSupportPoints). The
    // support tree steps deliberately do NOT read it - filtering happens once,
    // upstream, so an exported point list no longer carries points the tree
    // would have dropped for angle, and an imported list is never re-filtered
    // (capability sla-overhang-threshold-semantics). Do not delete this field
    // as unused: it is the only route the value travels from the config to
    // that filter.
    double overhang_angle_threshold = M_PI / 2;

    // The max length of a bridge in mm
    double max_bridge_length_mm = 10.0;

    // The max distance of a pillar to pillar link.
    double max_pillar_link_distance_mm = 10.0;

    // The elevation in Z direction upwards. This is the space between the pad
    // and the model object's bounding box bottom.
    double object_elevation_mm = 10;
    
    // The shortest distance between a pillar base perimeter from the model
    // body. This is only useful when elevation is set to zero.
    double pillar_base_safety_distance_mm = 0.5;
    
    unsigned max_bridges_on_pillar = 3;

    double max_weight_on_model_support = 10.f;

    double head_fullwidth() const {
        return 2 * head_front_radius_mm + head_width_mm +
               2 * head_back_radius_mm - head_penetration_mm;
    }

    double safety_distance() const { return safety_distance_mm; }
    double safety_distance(double r) const
    {
        return std::min(safety_distance_mm, r * safety_distance_mm / head_back_radius_mm);
    }

    // /////////////////////////////////////////////////////////////////////////
    // Compile time configuration values (candidates for runtime)
    // /////////////////////////////////////////////////////////////////////////

    // The max Z angle for a normal at which it will get completely ignored.
    static const double constexpr normal_cutoff_angle = 150.0 * M_PI / 180.0;

    // The safety gap between a support structure and model body. For support
    // struts smaller than head_back_radius, the safety distance is scaled
    // down accordingly. see method safety_distance()
    static const double constexpr safety_distance_mm = 0.5;

    static const double constexpr max_solo_pillar_height_mm = 15.0;
    static const double constexpr max_dual_pillar_height_mm = 35.0;
    static const double constexpr optimizer_rel_score_diff = 1e-10;
    static const unsigned constexpr optimizer_max_iterations = 2000;
    static const unsigned constexpr pillar_cascade_neighbors = 3;
    
};

enum class MeshType { Support, Pad };

struct SupportableMesh
{
    AABBMesh          emesh;
    SupportPoints     pts;
    SupportTreeConfig cfg;
    PadConfig         pad_cfg;
    double            zoffset = 0.;

    explicit SupportableMesh(const indexed_triangle_set &trmsh,
                             const SupportPoints        &sp,
                             const SupportTreeConfig    &c)
        : emesh{trmsh}, pts{sp}, cfg{c}
    {}

//    explicit SupportableMesh(const AABBMesh          &em,
//                             const SupportPoints     &sp,
//                             const SupportTreeConfig &c)
//        : emesh{em}, pts{sp}, cfg{c}
//    {}
};

// ---------------------------------------------------------------------------
// Per-point support geometry, resolved against the global configuration.
//
// A head ID is the index of the support point it grew from, by construction -
// see SupportTreeBuilder: "For heads it is beneficial to use the same IDs as
// for the support points". That is the whole link between a structure in the
// tree and the point that produced it. Structures with no originating point
// (the auxiliary pillars from interconnect_pillars(), anything addressed by
// pillar ID) pass SupportTreeNode::ID_UNSET and get the global value.
//
// DELIBERATE DIVERGENCE from the desktop PhrozenOrca: there, base_radius_mm
// and support_bracing_angle_deg only take effect when the point's
// type == manual_add (SupportTreeBuildsteps.cpp:495), while the other size
// fields carry no such gate. These resolvers apply every field on value alone
// and never look at SupportPointType. In the backend flow the caller edits
// mostly autogenerated island/slope points, so keeping the gate would make two
// of the seven fields silently do nothing with no error reported. This fork's
// tree never had the gate, so nothing is being removed here - the divergence
// is that none is added. Do not "restore" it when merging from the desktop
// side; see openspec change per-point-support-sizing, decision D8.
//
// Left out on purpose: values that normalize against the *global* reference
// radius rather than describe one point's geometry (safety_distance(),
// max_bridge_length scaling, pillar cascade distances). Those are ratios
// against the configured baseline, not per-point sizes. The pillar base of an
// auxiliary prop from interconnect_pillars() is global too - it grew from no
// support point.
//
// Not left out, and easy to miss: any geometry that compares two structures
// has to add up both of their resolved sizes rather than double one global
// value. The pillar clustering predicate in DefaultSupportTree::classify() is
// the one such place - it decides whether two pillar bases would overlap, so
// it sums the two points' base radii. A test written as "2 * the global
// radius" there would let custom bases intersect.
inline const SupportPoint *support_point_of(const SupportableMesh &sm, long head_id)
{
    return head_id >= 0 && size_t(head_id) < sm.pts.size() ?
               &sm.pts[size_t(head_id)] : nullptr;
}

inline double resolved_head_back_radius_mm(const SupportableMesh &sm, long head_id)
{
    const SupportPoint *sp = support_point_of(sm, head_id);
    return sp ? point_head_back_radius_mm(*sp, sm.cfg.head_back_radius_mm) :
                sm.cfg.head_back_radius_mm;
}

inline double resolved_head_width_mm(const SupportableMesh &sm, long head_id)
{
    const SupportPoint *sp = support_point_of(sm, head_id);
    return sp ? point_head_width_mm(*sp, sm.cfg.head_width_mm) :
                sm.cfg.head_width_mm;
}

inline double resolved_head_penetration_mm(const SupportableMesh &sm, long head_id)
{
    const SupportPoint *sp = support_point_of(sm, head_id);
    return sp ? point_head_penetration_mm(*sp, sm.cfg.head_penetration_mm) :
                sm.cfg.head_penetration_mm;
}

inline double resolved_base_radius_mm(const SupportableMesh &sm, long head_id)
{
    const SupportPoint *sp = support_point_of(sm, head_id);
    return sp ? point_base_radius_mm(*sp, sm.cfg.base_radius_mm) :
                sm.cfg.base_radius_mm;
}

inline double resolved_bridge_slope(const SupportableMesh &sm, long head_id)
{
    const SupportPoint *sp = support_point_of(sm, head_id);
    return sp ? point_bracing_angle_rad(*sp, sm.cfg.bridge_slope) :
                sm.cfg.bridge_slope;
}

// No resolver for contact_sphere_radius: this fork has no contact sphere
// geometry and no global contact sphere setting, so the field is carried
// through the data structures and the interchange format but has no consumer
// here (openspec change per-point-support-sizing, task 3.1).

// Is this surface tilted far enough downwards to carry a support head?
//
// polar     - the spherical polar angle of the SURFACE NORMAL, in radians, as
//             produced by Slic3r::Geometry::dir_to_spheric(): 0 means the
//             normal points straight up, PI means it points straight down.
// threshold - SupportTreeConfig::overhang_angle_threshold, in radians.
//
// Writing `s` for the surface's slope from the horizontal plane (0 for a flat
// down-facing face, PI/2 for a vertical wall), the two are related by
// polar = PI - s, so the test below rearranges to
//
//     s <= PI/2 - threshold          i.e.   slope <= 90deg - critical angle
//
// A SMALLER threshold therefore supports MORE surfaces: 0 supports every
// overhang, PI/2 supports only perfectly horizontal down-facing ones. This
// direction is frozen and matches both the support_critical_angle tooltip in
// PrintConfig.cpp and the four localisations of the DS-Online support angle
// hint; change one and all of them have to move together.
//
// DELIBERATE DIVERGENCE FROM PhrozenOrca. That codebase's
// sla_support_passes_overhang_filter() tests `slope <= critical angle`, i.e.
// the opposite scale, where a LARGER value supports more. The two agree only
// at 45 degrees. A merge between the projects must resolve this by hand and
// must NOT take either side's predicate verbatim
// (capability sla-overhang-threshold-semantics).
//
// The algebraically equivalent form `n.z() <= -sin(threshold)` is useful as an
// independent check in tests but is NOT the implementation: acos and sin need
// not agree on the last ulp right at the boundary.
//
// A NaN polar - reachable from a degenerate triangle, whose normal normalises
// to NaN and makes acos() return NaN - is REJECTED, because every comparison
// against NaN is false. That is the wanted answer: a point whose normal cannot
// be determined has no business carrying a head. Note it is also the opposite
// of what the old inline `if (polar < PI/2 + threshold) return;` did, which let
// a NaN fall through into head placement.
inline bool passes_overhang_filter(double polar, double threshold)
{
    return polar >= M_PI / 2.0 + threshold;
}

inline double ground_level(const SupportableMesh &sm)
{
    double lvl = sm.zoffset -
                 !bool(sm.pad_cfg.embed_object) * sm.cfg.enabled * sm.cfg.object_elevation_mm +
                  bool(sm.pad_cfg.embed_object) * sm.pad_cfg.wall_thickness_mm;

    return lvl;
}

indexed_triangle_set create_support_tree(const SupportableMesh &mesh,
                                         const JobController   &ctl);

indexed_triangle_set create_pad(const SupportableMesh      &model_mesh,
                                const indexed_triangle_set &support_mesh,
                                const JobController        &ctl);

std::vector<ExPolygons> slice(const indexed_triangle_set &support_mesh,
                              const indexed_triangle_set &pad_mesh,
                              const std::vector<float>   &grid,
                              float                       closing_radius,
                              const JobController        &ctl);

} // namespace sla
} // namespace Slic3r

#endif // SLASUPPORTTREE_HPP

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
#include <map>
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

// One pillar carried in from a previous generation.
//
// This is what makes support generation additive: a newly placed support can
// see the pillars already on the plate, brace to the ones within reach, and
// count them towards its own link budget so it does not grow redundant
// auxiliary props - while the existing geometry is never recomputed and comes
// back byte-identical, because it is never emitted at all.
//
// Only what bracing needs is carried: where the pillar stands, how thick it is,
// and how many connections it already has.
struct PriorPillar
{
    // Caller's handle, echoed back so it can be told what the new support
    // attached to. The engine never interprets it.
    long   id       = -1;
    Vec3d  endpoint = Vec3d::Zero(); // bottom, at ground level
    double height   = 0.;
    double r_start  = 0.;
    double r_end    = 0.;
    unsigned links   = 0;
    unsigned bridges = 0;
};

using PriorPillars = std::vector<PriorPillar>;

// What a generation attached itself to, and what that did to the pillars it
// attached to.
//
// A support that braces to a neighbour changes that neighbour's link count even
// though its geometry is untouched, and the caller has to carry the new count
// into the NEXT generation or the engine will keep bracing to a pillar that is
// already full. Reporting it is the difference between an additive flow that
// stays correct and one that drifts after a few supports.
struct PriorAttachment
{
    long     prior_id = -1; // the caller's handle for the pillar attached to
    unsigned links    = 0;  // its link count after this generation
    unsigned bridges  = 0;  // its bridge count after this generation
};

using PriorAttachments = std::vector<PriorAttachment>;

// The braces a generation grew to pillars carried in from earlier ones, keyed by
// the caller's handle for the pillar each reaches.
//
// Handed over separately from the support's own mesh because it belongs to both
// ends: grown now, but only meaningful while that pillar exists. Kept apart, it
// can be taken away on its own when the pillar goes - without regrowing the
// support it came with, which would move geometry the user already accepted.
using FrozenBraceMeshes = std::map<long, indexed_triangle_set>;

// ---------------------------------------------------------------------------
// The support tree as data rather than as triangles.
//
// merged_mesh() is nothing but get_mesh() over the builder's element lists, so
// handing the lists over is handing over the whole support. A caller that has
// them can draw it, point at ONE element of it, and take that element away -
// none of which triangle soup allows. A single STL per support answers "how
// does it look"; this answers "what is it made of", which is what an editor
// needs.
//
// Every element is world-space and in the engine's own frame (the mesh it was
// given, grounded), the same frame the exported STL is in.
struct TreeHead
{
    Vec3d  pos = Vec3d::Zero(); // the point on the model it holds up
    Vec3d  dir = Vec3d::Zero(); // away from the surface; the pin points along it
    double r_pin = 0., r_back = 0., width = 0., penetration = 0.;
};

struct TreePillar
{
    Vec3d    endpt   = Vec3d::Zero(); // bottom
    double   height  = 0.;
    double   r_start = 0., r_end = 0.;
    unsigned links   = 0, bridges = 0;
};

struct TreeJunction
{
    Vec3d  pos = Vec3d::Zero();
    double r   = 0.;
};

struct TreePedestal
{
    Vec3d  pos      = Vec3d::Zero();
    double height   = 0.;
    double r_bottom = 0., r_top = 0.;
};

// One BAR of bracing. The builder has always held them one per record; only the
// merged mesh loses the boundary between them.
struct TreeBridge
{
    Vec3d  startp = Vec3d::Zero(), endp = Vec3d::Zero();
    double r      = 0.;
    double end_r  = 0.; // equal to r unless it tapers (DiffBridge / Anchor)
    // The caller's handle for the pillar this bar reaches, when that pillar came
    // in from an earlier generation; -1 when both ends belong to this one. It is
    // what lets the caller drop this bar, and only this bar, when that support
    // goes away.
    long   reaches = -1;
};

struct SupportTreeElements
{
    std::vector<TreeHead>     heads;
    // Only the pillars this generation grew, in the same order as the
    // PriorPillars reported alongside - so the caller's handle for pillars[i]
    // is that list's [i].id, and there is no second numbering to keep in sync.
    std::vector<TreePillar>   pillars;
    std::vector<TreeJunction> junctions;
    std::vector<TreePedestal> pedestals;
    std::vector<TreeBridge>   bridges;
};

struct SupportableMesh
{
    AABBMesh          emesh;
    SupportPoints     pts;
    SupportTreeConfig cfg;
    PadConfig         pad_cfg;
    double            zoffset = 0.;

    // Pillars from earlier generations. Empty means a normal, from-scratch run.
    PriorPillars      prior;

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

inline double ground_level(const SupportableMesh &sm)
{
    double lvl = sm.zoffset -
                 !bool(sm.pad_cfg.embed_object) * sm.cfg.enabled * sm.cfg.object_elevation_mm +
                  bool(sm.pad_cfg.embed_object) * sm.pad_cfg.wall_thickness_mm;

    return lvl;
}

/// Grow the support tree.
/// @param out_pillars when given, receives the pillars this run created, so a
///        later run can be handed them as SupportableMesh::prior and brace to
///        them additively. Frozen pillars carried in are NOT repeated here:
///        the caller already has those.
indexed_triangle_set create_support_tree(const SupportableMesh &mesh,
                                         const JobController   &ctl,
                                         PriorPillars          *out_pillars = nullptr,
                                         PriorAttachments      *out_attached = nullptr,
                                         FrozenBraceMeshes     *out_braces = nullptr,
                                         SupportTreeElements   *out_elements = nullptr);

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

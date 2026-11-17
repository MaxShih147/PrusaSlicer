#ifndef SLA_BRACING_METRICS_HPP
#define SLA_BRACING_METRICS_HPP

// Measurement of the pillar-to-pillar bracing a Default support tree ended up
// with (openspec change sla-bracing-planar-topology, task group 1).
//
// It reads the builder directly instead of the exported tree: the baseline is
// taken before bars carry a `kind`, so the type of each bar is told by WHERE the
// builder keeps it (crossbridges vs bridges vs diffbridges) and by the owner ids
// written at creation time. Nothing here guesses a type from geometry.
//
// The rules mirror the spec's acceptance metrics so the very same function can
// be rerun after the change (task 7.1) and the numbers compared one to one.

#include <cstdint>
#include <string>
#include <vector>

#include "libslic3r/SLA/BridgeClearance.hpp"
#include "libslic3r/SLA/SupportTreeBuilder.hpp"

namespace Slic3r { namespace sla {

struct BracingMetrics
{
    // Clearance and angle limits the counts below are judged against.
    static constexpr double SAFETY_CLEARANCE_MM = clearance::SAFETY_CLEARANCE;
    static constexpr double MIN_SHARED_ANGLE_DEG = 20.;
    static constexpr double TALL_PILLAR_MM = 15.;

    size_t heads_valid   = 0;
    size_t pillars       = 0; // every pillar, frozen and auxiliary included
    size_t frozen        = 0;
    size_t aux_pillars   = 0; // props_for set: grown by interconnect_pillars()

    // Bars by where they came from.
    size_t truss_bars    = 0; // crossbridges: the zig-zag between two pillars
    size_t truss_groups  = 0; // distinct pillar pairs the truss bars join
    size_t orphan_truss  = 0; // truss bars missing an owner id (not grouped)
    size_t head_bars     = 0; // bridges reaching a pillar that is not a prop
    size_t prop_bars     = 0; // bridges reaching an auxiliary pillar
    size_t route_bars    = 0; // bridges of a route: no pillar owner, or the
                              // last one, ending on its pillar's top
    size_t route_diff    = 0; // diffbridges (routing, widening)

    // Bars whose recorded kind is missing, or differs from the kind told by
    // where the builder keeps them (task 2.2 check; both 0 when tagging works).
    size_t kind_unset     = 0;
    size_t kind_disagrees = 0;

    // Pillars whose axes are closer than r1 + r2 + c in plan view hug; those
    // hugging, directly or through others, form a cluster, which the rules
    // treat as one pillar (spec: 緊貼柱須視為同一叢集).
    size_t clusters      = 0; // clusters of two or more pillars
    size_t max_cluster   = 0; // pillars in the largest cluster

    // Groups the rescue or the safety net laid (Bridge::relaxed): judged bar
    // by bar in 3D, so their plan breaks are counted apart and not gated.
    size_t relaxed_groups = 0;
    size_t fallback_groups = 0; // ... of which the safety net laid

    // Plan-view rule breaks between truss groups, counted per offending pair,
    // neither of the two relaxed.
    size_t cross_truss   = 0; // no shared pillar / cluster, plan gap < r1 + r2 + c
    size_t shared_angle  = 0; // shared pillar / cluster, plan angle < 20 deg
    // ... the same where one of the two is relaxed.
    size_t cross_truss_relaxed  = 0;
    size_t shared_angle_relaxed = 0;
    // Truss groups passing a third pillar axis closer than r_bar + r_pillar + c,
    // counted per (group, pillar); pillars in an end's cluster are not third.
    // Relaxed groups are counted apart, in obstacle_pillar_relaxed.
    size_t obstacle_pillar = 0;
    size_t obstacle_pillar_relaxed = 0;
    // ... of which the pillar in the way does not stand on the ground (it sits
    // on the model, so the bar may well pass under or over it in 3D).
    size_t obstacle_pillar_offground = 0;
    size_t groups_in_plan_violation     = 0; // distinct groups in cross/angle
    size_t groups_in_obstacle_violation = 0; // distinct groups in obstacle

    // Non-frozen pillars taller than TALL_PILLAR_MM, and those of them with
    // links == 0. With no tall pillar at all the isolated count says nothing.
    size_t tall                 = 0;
    size_t isolated_tall        = 0;
    size_t isolated_tall_ground = 0; // ... of which stand on the ground

    // Pillars by how many distinct truss partners they have; the last bucket
    // holds "that many or more".
    std::vector<size_t> partners_histogram = std::vector<size_t>(6, 0);
    size_t max_partners = 0;

    std::string to_json() const;
};

/// `ground_z`: the tree's ground level, to tell ground pillars apart.
BracingMetrics bracing_metrics(const SupportTreeBuilder &builder, double ground_z);

/// FNV-1a 64 digests, printed as hex. Bit-level: any change to a coordinate,
/// radius, owner id or the order of the elements changes the digest.
struct BuilderDigests
{
    std::string heads, pillars, bridges, crossbridges, diffbridges, mesh;
    size_t      mesh_vertices = 0, mesh_faces = 0;
};

BuilderDigests builder_digests(const SupportTreeBuilder &builder);

}} // namespace Slic3r::sla

#endif // SLA_BRACING_METRICS_HPP

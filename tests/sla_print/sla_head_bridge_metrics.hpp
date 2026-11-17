#ifndef SLA_HEAD_BRIDGE_METRICS_HPP
#define SLA_HEAD_BRIDGE_METRICS_HPP

// Measurement of what the head routes of a Default support tree came out as
// (openspec change sla-head-bridge-clearance, task 6.4; design.md decision 11).
//
// The exploration scripts made formal: evidence/tools/analyze_tree.py counts
// the same things on an exported tree, where it has to trace which head a bar
// belongs to from the export. Here the builder is read directly, so every bar
// has its kind and owners, and a contact is judged by the engine's own rule
// (clearance::pair_contact(): the 3D axis distance against r1 + r2 + 0.2 mm,
// a pillar two elements share clipped out to its joint radius R first).
//
// Which head an element belongs to (its path) is traced the way the
// [head_routing] audit does it: a head bar by its head's own short pillar or
// by leaving the head's junction, a route by chaining on from the junction
// through route bars only, a pillar by the head that grew it, or the head whose
// route came down on it. The rules are analyze_tree.py's, count for count.
// Anchors are not read, as the export leaves them out too.

#include <cstddef>
#include <string>
#include <vector>

#include "libslic3r/SLA/BridgeClearance.hpp"
#include "libslic3r/SLA/RoutePlan.hpp"
#include "libslic3r/SLA/SupportTreeBuilder.hpp"

namespace Slic3r { namespace sla {

struct HeadBridgeMetrics
{
    static constexpr double SAFETY_CLEARANCE_MM = clearance::SAFETY_CLEARANCE;

    size_t heads_valid = 0;
    size_t pillars     = 0; // frozen and auxiliary included
    size_t aux_pillars = 0;
    size_t head_bars = 0, route_bars = 0, prop_bars = 0, truss_bars = 0;
    size_t unassigned  = 0; // head and route bars no head's path was found for

    // Pairs of elements closer than r1 + r2 + SAFETY_CLEARANCE_MM, one count
    // per pair; elements of one head's path never count against each other.
    // "path bar" is a head bar or a route bar.
    size_t A        = 0; // path bar x a path bar of another head
    size_t A_prop   = 0; // path bar x prop bar
    size_t B        = 0; // path bar x truss bar; a pillar of the truss's ends'
                         // clusters the path bar ends on is clipped out to its
                         // R first, the way the bracing planner judges it
    size_t B_shared = 0; // ... of which such a pillar was clipped
    size_t C_pillar = 0; // path bar x a pillar not its own end, auxiliary ones
                         // excluded (revC_aux)
    size_t C_head   = 0; // path bar x another head
    size_t revC_aux = 0; // an auxiliary pillar or a prop bar x a head, a path
                         // bar, a pillar or another prop bar (A_prop excluded)
    size_t revC_head = 0;        // pillar x a head it does not carry
    size_t revC_head_native = 0; // ... grown by a head (starts_from_head; not a
                                 // head sitting on it or a route coming down
                                 // on it) that is itself natively too close to
                                 // the head it touches (bare capsules):
                                 // support point spacing
    size_t revC_head_net = 0;    // ... the rest

    // Of A, C_pillar, C_head and revC_head_net: named by an L1, L2 or centre
    // pillar record of the same run (the record names the pair of heads, or a
    // head and a pillar no head's path owns, by that pillar's id), and the
    // rest. A_prop and revC_aux have no record to be named by, and B is
    // the bracing's (sla-bracing-topology): every one of those is unrecorded.
    size_t recorded   = 0;
    size_t unrecorded = 0;
    std::vector<std::string> unrecorded_list; // the first 20, for reading

    size_t D             = 0; // heads with more than one way down: head bars (from
                              // the junction or the head's own short pillar),
                              // route bars leaving the junction, and pillars
                              // the head grew that no head bar starts from
    size_t thick_on_thin = 0; // head bars onto a pillar thinner than the head
    size_t dup_truss_pairs = 0; // pillar pairs with one truss bar laid twice
    size_t pairs_excess  = 0; // sum over pillars of `pairs` beyond the pillars
                              // its truss bars reach; frozen pillars left out,
                              // their pairs count earlier generations' bracing

    // From the run's ClearanceReport.
    size_t l1 = 0, l2 = 0, none = 0;
    size_t centroid_recorded = 0; // centroid_pillar_clearance_recorded lines
    size_t centroid_native   = 0; // centre pillar contacts left out as spacing

    std::string to_json() const;
};

HeadBridgeMetrics head_bridge_metrics(const SupportTreeBuilder &builder,
                                      const ClearanceReport    &report);

}} // namespace Slic3r::sla

#endif // SLA_HEAD_BRIDGE_METRICS_HPP

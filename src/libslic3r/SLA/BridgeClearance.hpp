#ifndef SLA_BRIDGECLEARANCE_HPP
#define SLA_BRIDGECLEARANCE_HPP

// How close the parts of a support tree may come to each other in 3D: head
// bars, route bars, prop bars, pillars and pinheads, each taken as a capsule
// (a segment swept by a ball).
//
// This module is geometry only, like BracingPlanner: it knows capsules and the
// ids that tie them to pillars and heads, not meshes, trees or builders. The
// support tree feeds it and writes back the outcome, so the rules can be unit
// tested with no mesh at all, and the DS-online port has one unit to mirror
// (openspec change sla-head-bridge-clearance, design.md decisions 1 to 4).

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include <libslic3r/Point.hpp>

namespace Slic3r { namespace sla { namespace clearance {

// mm. The one clearance of the project: bracing::PlannerConfig takes its
// default from here. A physical limit of the print, not a setting.
inline constexpr double SAFETY_CLEARANCE = 0.2;

// mm: added to every query box, far above the rounding of coordinates of a
// few hundred mm, so the broad phase stays a superset in floating point.
inline constexpr double EPS_BROAD = 1e-6;
// What is left of a segment after clipping, in its own parameter, shorter than
// this is dropped: the clipped parts are closed, so a point left on the
// cylinder's surface counts as clipped.
inline constexpr double EPS_PIECE = 1e-9;

enum class PrimKind : uint8_t { head_bar, route_bar, prop_bar, pillar, pinhead };

// The names the support-tree export uses ("with" of a contact).
const char *kind_name(PrimKind k);

struct Capsule
{
    Vec3d  a = Vec3d::Zero(), b = Vec3d::Zero();
    double r = 0.;
};

// One element of a tree, committed or about to be.
struct Prim
{
    Capsule  cap;
    PrimKind kind     = PrimKind::head_bar;
    long     head_id  = -1; // the head whose path it is (a pillar: the head that grew it, or -1)
    long     pillar_a = -1; // bar: pillar at its start; pillar: its own id; pinhead: its carrier
    long     pillar_b = -1; // bar: pillar at its end
};

// One contact closer than allowed, for the log, the export and measurements.
struct Contact
{
    Vec3d    p_new = Vec3d::Zero(), p_old = Vec3d::Zero(); // closest points of the axes
    double   dist = 0., required = 0.;                     // axis distance, r1 + r2 + c
    PrimKind with = PrimKind::head_bar;                    // kind of the committed element
    long     with_head = -1, with_pillar = -1;             // its head_id; the pillar it stands on
                                                           // (a bar: pillar_b if set, else pillar_a)
    size_t   cand = 0;       // index into the candidates handed to violations()
    size_t   with_index = 0; // index into ClearanceIndex::prims()
};

// A shared pillar as the joint rule needs it: the axis in plan view and the
// radius r_P = max(r_start, r_end).
struct JointAxis
{
    Vec2d  xy = Vec2d::Zero();
    double r  = 0.;
};

// The axis of a pillar known by id. Asked only for pillars that two elements
// share, so every such id must be answerable.
using PillarAxisFn = std::function<JointAxis(long pillar_id)>;

// ─── elements to capsules (design.md decision 2) ─────────────────────────────

// Head bar, route Bridge, prop bar.
Capsule bar_capsule(const Vec3d &startp, const Vec3d &endp, double r);
// Route DiffBridge: the larger radius, which is never too optimistic.
Capsule diffbridge_capsule(const Vec3d &startp, const Vec3d &endp, double r, double end_r);
// Any pillar: foot to top, the larger of its radii.
Capsule pillar_capsule(const Vec3d &endpt, double height, double r_start, double r_end);
// A pinhead: from its tip `pos` to Head::junction_point(), radius r_back.
Capsule pinhead_capsule(const Vec3d &pos, const Vec3d &junction, double r_back);

// ─── distances ───────────────────────────────────────────────────────────────

// 3D distance of the axes (bracing::geom::seg_seg_dist3d); a zero-length axis
// is a point. Two capsules clear each other by c when this is >= x.r + y.r + c.
double capsule_dist(const Capsule &x, const Capsule &y);

// R_P = r_P + r1 + r2 + SAFETY_CLEARANCE. The joint radius keeps the full
// clearance whatever c the distance is judged by (spec "共用柱接點須以動態半徑
// R 豁免").
double joint_radius(double r_pillar, double r1, double r2);

// The closed parameter interval [t0, t1] of a(t) = a + t(b - a), t in [0, 1],
// that lies within plan distance R of `axis`. false when there is none, or
// only a single point of it (a tangent, an end exactly on the surface). A
// vertical segment (plan length squared <= bracing::geom::EPS_LEN) is judged
// by its ends: inside when both are, out when neither is, and split where the
// squared plan distance, taken as linear, crosses R^2 when they straddle it.
bool cylinder_interval(const Vec3d &a, const Vec3d &b, const Vec2d &axis, double R,
                       double &t0, double &t1);

// The parts of x outside every shared pillar's cylinder of radius
// joint_radius(P.r, x.r, r_other), as parameter intervals of x (at most
// shared.size() + 1). Empty when nothing is left.
struct Piece { double t0 = 0., t1 = 1.; };
std::vector<Piece> clip_joints(const Capsule &x, double r_other,
                               const std::vector<JointAxis> &shared);

// The axis distance of what is left of x and y after clip_joints(); +infinity
// when either is clipped away entirely. With no shared pillar, capsule_dist().
double joint_clipped_dist(const Capsule &x, const Capsule &y,
                          const std::vector<JointAxis> &shared);

// ─── the rules between two elements (design.md decision 4) ───────────────────

// Whether the pair is judged at all: pillar against pillar and pinhead against
// pinhead are not (non-goals of the change).
bool judged_kinds(PrimKind x, PrimKind y);
// The exemptions that skip a pair whole: the same head's path, a bar and a
// pillar it ends on, a pinhead and its carrier. By id only.
bool exempt(const Prim &x, const Prim &y);
// The pillars both elements stand on, by id: bar {pillar_a, pillar_b},
// pillar {its id}, pinhead {its carrier}; -1 is never one. Ascending.
std::vector<long> shared_pillars(const Prim &x, const Prim &y);

// The contact of a candidate with a committed element, judged with clearance
// c: none when the pair is not judged, exempt, or clear (exactly at the limit
// is clear). cand / with_index of the result are left 0.
bool pair_contact(const Prim &cand, const Prim &old, double c, const PillarAxisFn &axis_of,
                  Contact &out);

// ─── 3D index (design.md decision 3) ─────────────────────────────────────────

// The committed elements. Insert only: a committed element is never taken
// back. The R* tree is a broad phase that cannot miss a contact; every verdict
// is the 3D test of pair_contact().
class ClearanceIndex
{
    class Impl;
    std::unique_ptr<Impl> m_impl;

public:
    ClearanceIndex();
    ~ClearanceIndex();
    ClearanceIndex(ClearanceIndex &&) noexcept;
    ClearanceIndex &operator=(ClearanceIndex &&) noexcept;
    ClearanceIndex(const ClearanceIndex &) = delete;
    ClearanceIndex &operator=(const ClearanceIndex &) = delete;

    // Returns the element's index in prims().
    size_t insert(const Prim &p);
    // Names the pillar carrying the pinhead at prims()[i]. Pinheads go in when
    // the heads are made; which pillar carries one is known once its head is
    // routed. Only the id changes, never the capsule.
    void set_carrier(size_t i, long pillar_id);
    const std::vector<Prim> &prims() const;
    size_t size() const;

    // Every contact of the candidates with the committed elements at clearance
    // c (>= 0), ordered by (cand, with_index). Empty: the candidates pass. The
    // candidates are not judged against each other; they are one head's path.
    std::vector<Contact> violations(const std::vector<Prim> &cands, double c,
                                    const PillarAxisFn &axis_of) const;
};

}}} // namespace Slic3r::sla::clearance

#endif // SLA_BRIDGECLEARANCE_HPP

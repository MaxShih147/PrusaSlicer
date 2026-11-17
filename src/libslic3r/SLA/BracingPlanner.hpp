#ifndef SLA_BRACINGPLANNER_HPP
#define SLA_BRACINGPLANNER_HPP

// Which pillars get braced to which, and with which bars.
//
// The zig-zag bracing between two pillars always lies in the vertical plane
// above the line joining their axes. Two such groups can only touch if those
// lines come close in plan view, so the plan view is a broad phase that never
// misses a contact, and the few pairs it lets through get an exact 3D test.
//
// This module is geometry only: it knows pillars and bars, not meshes, trees or
// builders. Model collision is handed in as a callback, so the same decisions
// can be checked against test vectors with no mesh at all, and the DS-online
// port has one unit to mirror (openspec change sla-bracing-planar-topology,
// design.md decisions 1 to 7).

#include <algorithm>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

#include <libslic3r/Point.hpp>
#include <libslic3r/SLA/BridgeClearance.hpp>
#include <libslic3r/SLA/SpatIndex.hpp>
#include <libslic3r/SLA/SupportTreeStrategies.hpp>

namespace Slic3r { namespace sla { namespace bracing {

// Fixed by the spec, not configurable: they are physical limits of the print
// (resin trapped between bars, light bleeding across a gap), not preferences.
// The clearance itself is defined once, in BridgeClearance.hpp.
inline constexpr double   SAFETY_CLEARANCE_MM = clearance::SAFETY_CLEARANCE;
inline constexpr double   MIN_SHARED_ANGLE_DEG = 20.;
// cos(20 deg), written out rather than computed so the C++ engine and the
// JavaScript port compare against the very same number.
inline constexpr double   COS_MIN_SHARED_ANGLE = 0.9396926207859084;
inline constexpr unsigned PAIR_CAP = 3;
inline constexpr double   RESCUE_HEIGHT_MM = 15.;

// Why a candidate was or was not braced. The order of the checks is part of
// the contract: the first one that fails is the reason.
enum class Reason : uint8_t {
    accepted,
    too_close,       // plan distance below min_pillar_distance
    pair_cap,        // one end already braced to PAIR_CAP pillars
    obstacle_pillar, // the plan line passes a third pillar's axis
    shared_angle,    // too narrow an angle to a group sharing a pillar
    cross_truss,     // the plan line comes too close to another group
    z_clear_fail,    // every bar that fits is too close to another bar in 3D
    mesh_hit,        // every bar that fits hits the model
    no_room,         // the pillars leave no room for a single bar
    out_of_range,    // try_attach() only: farther than max_link_distance
    duplicate_pair   // attach() only: the pair is braced already
};

const char *reason_name(Reason r);

// What interconnect() needs to lay out the zig-zag, one to one.
struct TrussParams
{
    double               bridge_slope_rad = 0.; // positive
    double               ground_z         = 0.;
    double               base_height      = 0.;
    double               base_radius      = 0.; // decides crossing in dynamic mode
    PillarConnectionMode mode             = PillarConnectionMode::dynamic;
};

struct PlannerConfig
{
    double   safety_clearance    = clearance::SAFETY_CLEARANCE;
    double   cos_min_shared_ang  = COS_MIN_SHARED_ANGLE;
    unsigned pair_cap            = PAIR_CAP;
    double   rescue_height       = RESCUE_HEIGHT_MM;
    double   solo_height         = 15.;  // H1 of the links height rule
    double   min_height_ratio    = 0.5;
    double   max_link_distance   = 10.;
    double   min_pillar_distance = 1.;   // 2 x head_back_radius_mm
    TrussParams truss;
};

struct PillarGeom
{
    long     id = -1;          // the builder's pillar id
    Vec2d    xy = Vec2d::Zero(); // axis in plan view
    double   foot_z = 0.;      // endpt.z
    double   height = 0.;      // the top is foot_z + height, as Pillar::startpoint()
    double   r = 0.;           // r_start
    // max(r_start, r_end), for the joint radius R of a bar ending on it; 0
    // (or anything below r) when the pillar is not wider anywhere than r.
    double   r_joint = 0.;
    unsigned links = 0, pairs = 0;
    bool     frozen = false;
    // false: only in the way of others, never braced itself (a head's own
    // short pillar, which the old cascade never saw either).
    bool     braceable = true;

    double top_z() const { return foot_z + height; }
    double joint_r() const { return std::max(r, r_joint); }
};

struct Segment3
{
    Vec3d  a = Vec3d::Zero(), b = Vec3d::Zero();
    double r = 0.;
};

// A bar that is not bracing (head / prop / route, or one without a kind).
struct ObstacleBar
{
    Segment3 seg;
    // Pillars it joins. Where it ends on a pillar of the group being judged,
    // the joint rule of BridgeClearance clips both bars around that pillar.
    long     owner_a = -1, owner_b = -1;
};

struct CandidateEdge
{
    long   a = -1, b = -1; // a < b
    double dist = 0.;      // plan distance of the axes
};

struct BuiltTruss
{
    long   a = -1, b = -1;
    double r = 0.;
    std::vector<Segment3> bars;
    bool   rescued = false;
    bool   fallback = false; // laid by run_fallback()
    bool   existing = false; // handed in, not decided here
};

struct Decision
{
    CandidateEdge edge;
    Reason        reason = Reason::accepted;
    unsigned      bars_built = 0;
    bool          rescue = false;
    bool          fallback = false;
    int           truss = -1; // index into built() when accepted
};

// true when a bar of radius r from `from` to `to` does not hit the model.
using MeshClearFn = std::function<bool(const Vec3d &from, const Vec3d &to, double r)>;

class BracingPlanner
{
public:
    BracingPlanner(PlannerConfig cfg, MeshClearFn mesh_clear);

    void add_pillar(const PillarGeom &p);
    void add_obstacle_bar(const ObstacleBar &b);
    // A bracing bar that already exists between pillars a and b. Bars of the
    // same pair form one group; the pillars' counts are taken as given.
    void add_truss_bar(long a, long b, const Segment3 &bar);

    // Every candidate, shortest first; called once.
    std::vector<Decision> run_greedy();
    // Lonely tall pillars, tried once more with the bar-by-bar 3D rule in place
    // of the plan rule between groups. After run_greedy().
    std::vector<Decision> run_rescue();
    // Whether main and a prop not added yet could be braced. Changes nothing.
    // Bracing to a prop is not held back by the pair cap.
    Decision try_attach(long main_id, const PillarGeom &prop) const;
    // Brace two known pillars now, if the rules allow it (auxiliary pillars;
    // no pair cap either). A pair braced already is refused, duplicate_pair.
    Decision attach(long a, long b);
    // The candidates of the given (new) pillars only, shortest first.
    std::vector<Decision> run_greedy_for(const std::vector<long> &new_ids);
    // Tall pillars still without a link after everything else: braced to the
    // nearest pillar that gives them one, with the pair cap judged on links.
    std::vector<Decision> run_fallback();

    const std::vector<BuiltTruss> &built() const { return m_built; }
    const PillarGeom &pillar(long id) const;
    bool has_pillar(long id) const { return m_pillar_at.count(id) > 0; }
    // Pillars hugging each other (plan gap of the axes under r1 + r2 +
    // clearance), and pillars hugging those, are one cluster: the same value.
    long cluster_of(long id) const;

private:
    // Which rules a candidate is judged by.
    enum class Mode { greedy, attach, rescue, fallback };

    struct Evaluation
    {
        Reason                reason = Reason::accepted;
        std::vector<Segment3> bars; // the bars that passed
    };

    PillarGeom       &pillar_mut(long id);
    std::vector<CandidateEdge> collect(const std::vector<long> &from) const;
    Evaluation evaluate(const PillarGeom &pa, const PillarGeom &pb, double dist,
                        Mode mode) const;
    int  accept(const CandidateEdge &e, std::vector<Segment3> &&bars, Mode mode);
    bool rescue_target(const PillarGeom &p) const;
    bool gives_link(const PillarGeom &p, const PillarGeom &other) const;
    bool built_pair(long a, long b) const;

    size_t find_root(size_t idx) const;
    void   unite(size_t i, size_t j);
    // The clusters p stands in: its own, or for a pillar not added yet, those
    // of the pillars it would hug.
    std::vector<size_t> roots_of(const PillarGeom &p) const;
    long root_of_id(long id) const; // -1 when not a known pillar

    PlannerConfig m_cfg;
    MeshClearFn   m_mesh_clear;

    std::vector<PillarGeom>            m_pillars;
    std::unordered_map<long, size_t>   m_pillar_at;
    // Union-find over m_pillars indices, and the members of every root.
    std::vector<size_t>                m_parent;
    std::unordered_map<size_t, std::vector<size_t>> m_members;
    std::vector<ObstacleBar>           m_obstacles;
    std::vector<BuiltTruss>            m_built;
    std::unordered_map<long, std::vector<int>> m_truss_of; // pillar -> groups
    std::vector<CandidateEdge>         m_candidates;       // from run_greedy()
    bool                               m_greedy_done = false;

    // Plan-view R* trees. Queries only ask "is anything in the way", so the
    // order they return elements in never shows in a decision.
    mutable BoxIndex m_pillar_idx, m_truss_idx, m_bar_idx;
};

// Pure functions: unit tested on their own, and the one-to-one targets of the
// JavaScript port.
namespace geom {

inline constexpr double EPS_LEN = 1e-12; // mm^2: a segment shorter is a point
inline constexpr double EPS_PAR = 1e-12; // relative: below this, parallel

double point_seg_dist2d(const Vec2d &p, const Vec2d &a, const Vec2d &b);
double seg_seg_dist2d(const Vec2d &a, const Vec2d &b, const Vec2d &c, const Vec2d &d);
// The same, also giving the parameters of the closest points on [a,b] and [c,d].
double seg_seg_closest2d(const Vec2d &a, const Vec2d &b, const Vec2d &c,
                         const Vec2d &d, double &s, double &t);
// Ericson, Real-Time Collision Detection, 5.1.9.
double seg_seg_dist3d(const Vec3d &p1, const Vec3d &q1, const Vec3d &p2, const Vec3d &q2);
// The same, also giving the parameters of the closest points on [p1,q1] and
// [p2,q2]. seg_seg_dist3d() is this, so the two never disagree.
double seg_seg_closest3d(const Vec3d &p1, const Vec3d &q1, const Vec3d &p2,
                         const Vec3d &q2, double &s, double &t);
// Height of bar 1 minus height of bar 2 where they come closest in plan view.
// Diagnostics only: the 3D distance, not this, decides.
double plan_closest_dz(const Segment3 &bar1, const Segment3 &bar2);
// The angle at `shared` between the directions to end1 and end2 is at least
// the one whose cosine is cos_min. No acos: the cosines are compared.
bool shared_angle_ok(const Vec2d &shared, const Vec2d &end1, const Vec2d &end2,
                     double cos_min);
// The same for two directions that need not start at one point: groups
// leaving one cluster from different pillars of it.
bool dir_angle_ok(const Vec2d &u, const Vec2d &v, double cos_min);
// The zig-zag interconnect() lays between p and q, without the model check.
std::vector<Segment3> truss_bars(const PillarGeom &p, const PillarGeom &q,
                                 const TrussParams &prm);

} // namespace geom

}}} // namespace Slic3r::sla::bracing

#endif // SLA_BRACINGPLANNER_HPP

///|/ Copyright (c) Prusa Research 2020 - 2023 Tomáš Mészáros @tamasmeszaros, Lukáš Hejl @hejllukas
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef LEGACYSUPPORTTREE_HPP
#define LEGACYSUPPORTTREE_HPP

#include <libslic3r/SLA/SpatIndex.hpp>
#include <libslic3r/Execution/ExecutionTBB.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <map>
#include <set>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include "SupportTreeUtilsLegacy.hpp"
#include "libslic3r/AABBMesh.hpp"
#include "libslic3r/Execution/Execution.hpp"
#include "libslic3r/SLA/BracingPlanner.hpp"
#include "libslic3r/SLA/Pad.hpp"
#include "libslic3r/SLA/RoutePlan.hpp"
#include "libslic3r/SLA/SupportTree.hpp"
#include "libslic3r/SLA/SupportTreeBuilder.hpp"
#include "libslic3r/Point.hpp"

namespace Slic3r { namespace sla {

inline constexpr const auto &suptree_ex_policy = ex_tbb;

class PillarIndex {
    PointIndex m_index;
    using Mutex = execution::BlockingMutex<decltype(suptree_ex_policy)>;
    mutable Mutex m_mutex;

public:

    template<class...Args> inline void guarded_insert(Args&&...args)
    {
        std::lock_guard<Mutex> lck(m_mutex);
        m_index.insert(std::forward<Args>(args)...);
    }

    template<class...Args>
    inline std::vector<PointIndexEl> guarded_query(Args&&...args) const
    {
        std::lock_guard<Mutex> lck(m_mutex);
        return m_index.query(std::forward<Args>(args)...);
    }

    template<class...Args> inline void insert(Args&&...args)
    {
        m_index.insert(std::forward<Args>(args)...);
    }

    template<class...Args>
    inline std::vector<PointIndexEl> query(Args&&...args) const
    {
        return m_index.query(std::forward<Args>(args)...);
    }

    template<class Fn> inline void foreach(Fn fn) { m_index.foreach(fn); }
    template<class Fn> inline void guarded_foreach(Fn fn)
    {
        std::lock_guard<Mutex> lck(m_mutex);
        m_index.foreach(fn);
    }

    PointIndex guarded_clone()
    {
        std::lock_guard<Mutex> lck(m_mutex);
        return m_index;
    }
};

class DefaultSupportTree {
    const SupportableMesh &m_sm;

    using PtIndices = std::vector<unsigned>;

    PtIndices m_iheads;            // support points with pinhead
    PtIndices m_iheads_onmodel;

    std::map<unsigned, AABBMesh::hit_result> m_head_to_ground_scans;

    // normals for support points from model faces.
    Eigen::MatrixXd  m_support_nmls;

    // Clusters of points which can reach the ground directly and can be
    // bridged to one central pillar
    std::vector<PtIndices> m_pillar_clusters;

    // This algorithm uses the SupportTreeBuilder class to fill gradually
    // the support elements (heads, pillars, bridges, ...)
    SupportTreeBuilder& m_builder;

    // support points in Eigen/IGL format
    Eigen::MatrixXd m_points;

    // Builder ids of the pillars adopted from SupportableMesh::prior, and the
    // caller handle each one came in with.
    std::vector<long> m_prior_pillar_ids;
    std::map<long, long> m_prior_caller_ids; // builder id -> caller id

    // Which prior pillars this run attached to. Recorded rather than derived:
    // only the code that succeeds in bracing knows it happened.
    std::set<long> m_attached_prior_ids;

    // throw if canceled: It will be called many times so a shorthand will
    // come in handy.
    ThrowOnCancel m_thr;

    // A spatial index to easily find strong pillars to connect to.
    PillarIndex m_pillar_index;

    // How many thin wall penetration measurements found no exit surface. Filled
    // from add_pinheads(), which runs its filter across worker threads, so the
    // counter inside is atomic.
    PenetrationClampStats m_penetration_stats;

    inline AABBMesh::hit_result ray_mesh_intersect(const Vec3d& s,
                                                      const Vec3d& dir)
    {
        return m_sm.emesh.query_ray_hit(s, dir);
    }

    // This function will test if a future pinhead would not collide with the
    // model geometry. It does not take a 'Head' object because those are
    // created after this test. Parameters: s: The touching point on the model
    // surface. dir: This is the direction of the head from the pin to the back
    // r_pin, r_back: the radiuses of the pin and the back sphere width: This
    // is the full width from the pin center to the back center m: The object
    // mesh.
    // The return value is the hit result from the ray casting. If the starting
    // point was inside the model, an "invalid" hit_result will be returned
    // with a zero distance value instead of a NAN. This way the result can
    // be used safely for comparison with other distances.
    AABBMesh::hit_result pinhead_mesh_intersect(
        const Vec3d& s,
        const Vec3d& dir,
        double r_pin,
        double r_back,
        double width,
        double safety_d);

    AABBMesh::hit_result pinhead_mesh_intersect(const Vec3d &s,
                                                   const Vec3d &dir,
                                                   double       r_pin,
                                                   double       r_back,
                                                   double       width)
    {
        return pinhead_mesh_intersect(s, dir, r_pin, r_back, width,
                                      r_back * m_sm.cfg.safety_distance_mm /
                                          m_sm.cfg.head_back_radius_mm);
    }

    // Checking bridge (pillar and stick as well) intersection with the model.
    // If the function is used for headless sticks, the ins_check parameter
    // have to be true as the beginning of the stick might be inside the model
    // geometry.
    // The return value is the hit result from the ray casting. If the starting
    // point was inside the model, an "invalid" hit_result will be returned
    // with a zero distance value instead of a NAN. This way the result can
    // be used safely for comparison with other distances.
    AABBMesh::hit_result bridge_mesh_intersect(
        const Vec3d& s,
        const Vec3d& dir,
        double r,
        double safety_d) const;

    AABBMesh::hit_result bridge_mesh_intersect(
        const Vec3d& s,
        const Vec3d& dir,
        double r) const
    {
        return bridge_mesh_intersect(s, dir, r,
                                     r * m_sm.cfg.safety_distance_mm /
                                         m_sm.cfg.head_back_radius_mm);
    }

    template<class...Args>
    inline double bridge_mesh_distance(Args&&...args) const {
        return bridge_mesh_intersect(std::forward<Args>(args)...).distance();
    }

    // Writes the zig-zag bars of a group the BracingPlanner accepted between
    // two pillars, tagged as truss. Returns the number of bars written.
    unsigned interconnect(const bracing::BuiltTruss &truss);

    // ─── routing a head: planned read only, then committed whole ─────────────
    //
    // Steps 3(b) and 4 route their heads in two phases (route_heads()). Phase
    // one plans every head at once against the state the step started from;
    // phase two commits one route per head, in head id order, on one thread.
    // The plan_* functions only read; commit() is the only writer.

    // What planning reads: the builder, and the feet of the pillars a head may
    // bridge to, copied out of the pillar index.
    struct RouteView
    {
        const SupportTreeBuilder &builder;
        std::vector<PointIndexEl> feet;
    };
    RouteView route_view();

    // The rules no route may break (design.md decision 8), on the pillar's
    // state in `builder`: room for one more bridge, and no head on a pillar
    // thinner than its back. The one place the bridge cap is compared.
    bool can_bridge_to(const SupportTreeBuilder &builder, const Pillar &pillar,
                       const Head &head) const;

    // How long a bar from this head may be: max_bridge_length_mm, scaled by
    // the head's back radius against the global one.
    double bridge_reach(const Head &head) const
    {
        return head.r_back_mm * m_sm.cfg.max_bridge_length_mm / m_sm.cfg.head_back_radius_mm;
    }

    // A bar from the head to the pillar, with a short pillar of the head's own
    // under it where the bar cannot leave the junction itself.
    std::optional<RoutePlan> plan_to_pillar(const SupportTreeBuilder &builder,
                                            const Head &head, long pillar_id) const;

    // to_pillar plans to the pillars in the view other than `skip`, nearest
    // foot first (ties by pillar id), until `out` holds `limit` of them.
    void plan_near_pillars(const RouteView &view, const Head &head, long skip, size_t limit,
                           std::vector<RoutePlan> &out) const;

    // This is a proxy function for pillar creation which will mind the gap
    // between the pad and the model bottom in zero elevation mode.
    // jp is the starting junction point which needs to be routed down.
    // sourcedir is the allowed direction of an optional bridge between the
    // jp junction and the final pillar.
    std::optional<RoutePlan> plan_ground_pillar(const Junction &jp, const Vec3d &sourcedir,
                                                long head_id) const;

    // A route for the head to the ground: a bar to where the way down is
    // clear, then a pillar. plan_ground_route() tries the bar along the head;
    // plan_ground_detour() searches a direction out of the cavity, for when
    // that failed (see sla::plan_ground_detour() on running it in parallel).
    std::optional<RoutePlan> plan_ground_route(const Head &head) const;
    std::optional<RoutePlan> plan_ground_detour(const Head &head) const;

    // A pillar from the head down to the model, standing on an anchor.
    std::optional<RoutePlan> plan_model_anchor(const Head &head) const;

    enum class RouteStep { ground, model }; // step 3(b), step 4

    // A head's candidates in the order they are tried: to_pillar, then the
    // fallbacks (step 3(b): own pillar; step 4: ground route, model anchor).
    struct HeadRoutes
    {
        std::vector<RoutePlan> to_pillar, fallback;
        bool fallback_planned = false;
    };

    // Plans the head's candidates against the view. The pillar of the head's
    // cluster, `center`, comes first when there is one. Fallbacks are planned
    // only if no pillar is in reach, or on a recompute; they read the mesh
    // alone, so once planned they stay valid and are kept.
    void plan_routes(RouteStep step, const RouteView &view, const Head &head, long center,
                     bool recompute, HeadRoutes &routes) const;

    // Makes the plan in the builder and indexes its new pillar. Returns that
    // pillar's builder id, or ID_UNSET when the plan adds none to the index.
    long commit(const RoutePlan &plan);

    struct RouteJob { unsigned head; long center; };
    struct RouteCounts { size_t routed = 0, recomputed = 0, unrouted = 0; };

    // The two phases of a routing step, for the heads of `jobs`.
    RouteCounts route_heads(RouteStep step, std::vector<RouteJob> jobs);

    // Plans and commits a ground pillar at once, for the cluster centres of
    // step 3(a). Returns the pillar's id, or ID_UNSET.
    long create_ground_pillar(const Junction &jp,
                              const Vec3d &sourcedir,
                              long         head_id = SupportTreeNode::ID_UNSET);

    // Stays on the global base radius on purpose: pid here is a *pillar* id,
    // not a head id, and the only caller is interconnect_pillars(), which adds
    // auxiliary props that grew from no support point. Per-point base radii
    // are applied in create_ground_pillar(), where the originating point is
    // known.
    void add_pillar_base(long pid)
    {
        m_builder.add_pillar_base(pid, m_sm.cfg.base_height_mm, m_sm.cfg.base_radius_mm);
    }

    std::optional<DiffBridge> search_widening_path(const Vec3d &jp,
                                                   const Vec3d &dir,
                                                   double       radius,
                                                   double       new_radius)
    {
        return sla::search_widening_path(suptree_ex_policy, m_sm, jp, dir, radius, new_radius);
    }

public:
    DefaultSupportTree(SupportTreeBuilder & builder, const SupportableMesh &sm);

    // Now let's define the individual steps of the support generation algorithm

    // Filtering step: here we will discard inappropriate support points
    // and decide the future of the appropriate ones. We will check if a
    // pinhead is applicable and adjust its angle at each support point. We
    // will also merge the support points that are just too close and can
    // be considered as one.
    void add_pinheads();

    // Further classification of the support points with pinheads. If the
    // ground is directly reachable through a vertical line parallel to the
    // Z axis we consider a support point as pillar candidate. If touches
    // the model geometry, it will be marked as non-ground facing and
    // further steps will process it. Also, the pillars will be grouped
    // into clusters that can be interconnected with bridges. Elements of
    // these groups may or may not be interconnected. Here we only run the
    // clustering algorithm.
    void classify();

    // Step: Routing the ground connected pinheads, and interconnecting
    // them with additional (angled) bridges. Not all of these pinheads
    // will be a full pillar (ground connected). Some will connect to a
    // nearby pillar using a bridge. The max number of such side-heads for
    // a central pillar is limited to avoid bad weight distribution.
    void routing_to_ground();

    // Step: routing the pinheads that would connect to the model surface
    // along the Z axis downwards. For now these will actually be connected with
    // the model surface with a flipped pinhead. In the future here we could use
    // some smart algorithms to search for a safe path to the ground or to a
    // nearby pillar that can hold the supported weight.
    void routing_to_model();

    // Register SupportableMesh::prior into the builder and the spatial index,
    // so a newly placed support can brace to what is already on the plate
    // without any of it being recomputed or re-emitted.
    void adopt_prior_pillars();

    /// Note that this run braced to a prior pillar, if that is what `pillar_id`
    /// is. Safe to call for any pillar; only frozen ones are recorded.
    void note_prior_attachment(long pillar_id);

    /// What this run attached to, with the link counts those pillars now carry.
    PriorAttachments prior_attachments() const;

    void interconnect_pillars();

    inline void merge_result() { m_builder.merged_mesh(); }

    /// @param out_attached when given, receives the prior pillars this run
    ///        braced to, with the link counts they now carry.
    static bool execute(SupportTreeBuilder & builder, const SupportableMesh &sm,
                        PriorAttachments *out_attached = nullptr);
};

inline void create_default_tree(SupportTreeBuilder &builder, const SupportableMesh &sm)
{
    DefaultSupportTree::execute(builder, sm);
}

}} // namespace Slic3r::sla

#endif // LEGACYSUPPORTTREE_HPP

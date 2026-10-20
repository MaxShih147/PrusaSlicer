///|/ Copyright (c) Prusa Research 2022 - 2023 Tomáš Mészáros @tamasmeszaros, Vojtěch Bubník @bubnikv
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "DefaultSupportTree.hpp"

#include <libslic3r/Optimize/NLoptOptimizer.hpp>
#include <libslic3r/SLA/Clustering.hpp>
#include <libslic3r/MeshNormals.hpp>
#include <libslic3r/Execution/ExecutionTBB.hpp>
#include <boost/log/trivial.hpp>
#include <oneapi/tbb/task_arena.h>
#include <cstdlib>
#include <cmath>
#include <functional>
#include <iterator>
#include <set>
#include <string>
#include <cassert>
#include <cstddef>

#include "libslic3r/Geometry.hpp"
#include "libslic3r/Optimize/Optimizer.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/SLA/SpatIndex.hpp"
#include "libslic3r/SLA/SupportPoint.hpp"
#include "libslic3r/SLA/SupportTreeStrategies.hpp"
#include "libslic3r/SLA/SupportTreeUtils.hpp"
#include "libslic3r/SLA/SupportTreeUtilsLegacy.hpp"
#include "libslic3r/libslic3r.h"

namespace Slic3r { namespace sla {

using Slic3r::opt::initvals;
using Slic3r::opt::bounds;
using Slic3r::opt::StopCriteria;
using Slic3r::opt::Optimizer;
using Slic3r::opt::AlgNLoptSubplex;
using Slic3r::opt::AlgNLoptGenetic;

DefaultSupportTree::DefaultSupportTree(SupportTreeBuilder &   builder,
                                     const SupportableMesh &sm)
    : m_sm(sm)
    , m_support_nmls(sm.pts.size(), 3)
    , m_builder(builder)
    , m_points(sm.pts.size(), 3)
    , m_thr(builder.ctl().cancelfn)
{
    // Prepare the support points in Eigen/IGL format as well, we will use
    // it mostly in this form.

    long i = 0;
    for (const SupportPoint &sp : m_sm.pts) {
        m_points.row(i).x() = double(sp.pos.x());
        m_points.row(i).y() = double(sp.pos.y());
        m_points.row(i).z() = double(sp.pos.z());
        ++i;
    }
}

bool DefaultSupportTree::execute(SupportTreeBuilder    &builder,
                                const SupportableMesh &sm,
                                PriorAttachments      *out_attached,
                                ClearanceReport       *out_clearance)
{
    if(sm.pts.empty()) return false;

    DefaultSupportTree alg(builder, sm);

    // Seed the pillars carried in from earlier generations. They go into the
    // builder so interconnect() can take them by reference, and into the
    // spatial index so neighbour queries find them - but they are flagged
    // frozen, so merged_mesh() never emits their geometry and the caller's
    // existing support mesh stays byte-identical.
    alg.adopt_prior_pillars();

       // Let's define the individual steps of the processing. We can experiment
       // later with the ordering and the dependencies between them.
    enum Steps {
        BEGIN,
        PINHEADS,
        CLASSIFY,
        ROUTING_GROUND,
        ROUTING_NONGROUND,
        CASCADE_PILLARS,
        MERGE_RESULT,
        DONE,
        ABORT,
        NUM_STEPS
        //...
    };

       // Collect the algorithm steps into a nice sequence
    std::array<std::function<void()>, NUM_STEPS> program = {
        [] () {
            // Begin...
            // Potentially clear up the shared data (not needed for now)
        },

        std::bind(&DefaultSupportTree::add_pinheads, &alg),

        std::bind(&DefaultSupportTree::classify, &alg),

        std::bind(&DefaultSupportTree::routing_to_ground, &alg),

        std::bind(&DefaultSupportTree::routing_to_model, &alg),

        std::bind(&DefaultSupportTree::interconnect_pillars, &alg),

        std::bind(&DefaultSupportTree::merge_result, &alg),

        [] () {
            // Done
        },

        [] () {
            // Abort
        }
    };

    Steps pc = BEGIN;

    if(sm.cfg.ground_facing_only) {
        program[ROUTING_NONGROUND] = []() {
            BOOST_LOG_TRIVIAL(info)
                << "Skipping model-facing supports as requested.";
        };
    }

       // Let's define a simple automaton that will run our program.
    auto progress = [&builder, &pc] () {
        static const std::array<std::string, NUM_STEPS> stepstr {
            "Starting",
            "Generate pinheads",
            "Classification",
            "Routing to ground",
            "Routing supports to model surface",
            "Interconnecting pillars",
            "Merging support mesh",
            "Done",
            "Abort"
        };

        static const std::array<unsigned, NUM_STEPS> stepstate {
            0,
            30,
            50,
            60,
            70,
            80,
            99,
            100,
            0
        };

        if(builder.ctl().stopcondition()) pc = ABORT;

        switch(pc) {
        case BEGIN: pc = PINHEADS; break;
        case PINHEADS: pc = CLASSIFY; break;
        case CLASSIFY: pc = ROUTING_GROUND; break;
        case ROUTING_GROUND: pc = ROUTING_NONGROUND; break;
        case ROUTING_NONGROUND: pc = CASCADE_PILLARS; break;
        case CASCADE_PILLARS: pc = MERGE_RESULT; break;
        case MERGE_RESULT: pc = DONE; break;
        case DONE:
        case ABORT: break;
        default: ;
        }

        builder.ctl().statuscb(stepstate[pc], stepstr[pc]);
    };

       // Just here we run the computation...
    while(pc < DONE) {
        progress();
        program[pc]();
    }

    // Both the pinhead pass and the model-body anchors report into the same
    // counter, so one line covers the whole tree. Skipped on abort: the run was
    // cancelled, the numbers are partial, and the user is not waiting on a
    // diagnostic for work they just stopped.
    if (pc != ABORT) {
        report_penetration_failsafes(alg.m_penetration_stats,
                                     "Default support tree");
        report_route_clearance(alg.m_clearance_report, "Default support tree");
    }

    if (out_attached)
        *out_attached = alg.prior_attachments();
    if (out_clearance)
        *out_clearance = std::move(alg.m_clearance_report);

    return pc == ABORT;
}

AABBMesh::hit_result DefaultSupportTree::pinhead_mesh_intersect(
    const Vec3d &s,
    const Vec3d &dir,
    double       r_pin,
    double       r_back,
    double       width,
    double       sd)
{
    return sla::pinhead_mesh_hit(suptree_ex_policy, m_sm.emesh, s, dir, r_pin, r_back, width, sd);
}

AABBMesh::hit_result DefaultSupportTree::bridge_mesh_intersect(
    const Vec3d &src, const Vec3d &dir, double r, double sd) const
{
    return sla::beam_mesh_hit(suptree_ex_policy, m_sm.emesh, {src, dir, r}, sd);
}

unsigned DefaultSupportTree::interconnect(const bracing::BuiltTruss &truss)
{
    // The planner has laid the zig-zag out (geom::truss_bars, the layout this
    // function used to compute) and judged it bar by bar: against the model,
    // through the callback built in interconnect_pillars(), and against every
    // other bar. Here the bars that passed are written down, as truss
    // crossbridges between the two pillars, each as thick as the thinner one.
    // Groups the rescue or the safety net laid were judged bar by bar in 3D
    // rather than by the plan rule; the bars say so.
    const bool relaxed = truss.rescued || truss.fallback;
    for (const bracing::Segment3 &bar : truss.bars)
        m_builder.add_crossbridge_between(truss.a, truss.b, bar.a, bar.b, bar.r, relaxed,
                                          truss.fallback);

    return unsigned(truss.bars.size());
}

namespace {

RoutePlan pillar_route(RoutePlan::Kind kind, PillarRecipe &&planned)
{
    RoutePlan plan;
    plan.kind       = kind;
    plan.new_pillar = planned.pillar;
    plan.recipe     = std::move(planned.recipe);
    return plan;
}

} // namespace

DefaultSupportTree::RouteView DefaultSupportTree::route_view()
{
    RouteView view{m_builder, {}};
    m_pillar_index.foreach([&view](const PointIndexEl &el) { view.feet.emplace_back(el); });
    return view;
}

bool DefaultSupportTree::can_bridge_to(const SupportTreeBuilder &builder,
                                       const Pillar             &pillar,
                                       const Head               &head) const
{
    return sla::can_bridge_to(builder, pillar, head, m_sm.cfg.max_bridges_on_pillar);
}

std::optional<RoutePlan> DefaultSupportTree::plan_to_pillar(const SupportTreeBuilder &builder,
                                                            const Head               &head,
                                                            long nearpillar_id) const
{
    // Nothing is written while routes are planned, so the reference holds.
    const Pillar &nearpillar = builder.pillar(nearpillar_id);

    // First, as they cost nothing. A bridge count only grows, so a pillar
    // that is full now is full at commit too, where this is asked again.
    if (!can_bridge_to(builder, nearpillar, head))
        return {};

    Vec3d headjp = head.junction_point();
    Vec3d nearjp_u = nearpillar.startpoint();
    Vec3d nearjp_l = nearpillar.endpoint();

    double r = head.r_back_mm;
    double d2d = distance(to_2d(headjp), to_2d(nearjp_u));
    double d3d = distance(headjp, nearjp_u);

    double hdiff = nearjp_u.z() - headjp.z();
    double slope = std::atan2(hdiff, d2d);

    Vec3d bridgestart = headjp;
    Vec3d bridgeend = nearjp_u;
    double max_len = bridge_reach(head);

    // No bar from here reaches that pillar: both lengths tested below, d3d
    // and D, run from the junction to the pillar's axis and are never shorter
    // than d2d. Most pillars of a model fail here, before any mesh test.
    if (d2d > max_len) return {};

    // This head's own bracing angle. Most heads in a cluster reach the ground
    // through this bridge rather than through their own pillar, so leaving the
    // global value here would make a custom angle do nothing for every head
    // except the cluster centroid.
    double max_slope = resolved_bridge_slope(m_sm, head.id);
    double zdiff = 0.0;

       // check the default situation if feasible for a bridge
    if(d3d > max_len || slope > -max_slope) {
        // not feasible to connect the two head junctions. We have to search
        // for a suitable touch point.

        double Zdown = headjp.z() + d2d * std::tan(-max_slope);
        Vec3d touchjp = bridgeend; touchjp.z() = Zdown;
        double D = distance(headjp, touchjp);
        zdiff = Zdown - nearjp_u.z();

        if(zdiff > 0) {
            Zdown -= zdiff;
            bridgestart.z() -= zdiff;
            touchjp.z() = Zdown;
        }

        if(Zdown <= nearjp_u.z() && Zdown >= nearjp_l.z() && D < max_len)
            bridgeend.z() = Zdown;
        else
            return {};
    }

       // There will be a minimum distance from the ground where the
       // bridge is allowed to connect. This is an empiric value.
    double minz = ground_level(m_sm) + 4 * head.r_back_mm;
    if(bridgeend.z() < minz) return {};

    // The mesh last, as it costs the most. Every test here only turns the
    // pillar down, so their order does not change which pillars pass.

       // We can't insert a pillar under the source head to connect
       // with the nearby pillar's starting junction
    if(zdiff > 0 && bridge_mesh_distance(headjp, DOWN, r) < zdiff) return {};

    double t = bridge_mesh_distance(bridgestart, dirv(bridgestart, bridgeend), r);

       // Cannot insert the bridge. (further search might not worth the hassle)
    if(t < distance(bridgestart, bridgeend)) return {};

    RoutePlan plan;
    plan.kind          = RoutePlan::Kind::to_pillar;
    plan.target_pillar = nearpillar_id;

    BuildRecipe &recipe = plan.recipe;
    if(zdiff > 0) {
        // A partial pillar is needed under the starting head. It is this
        // head's own, and the bar then runs from ITS top across to the near
        // pillar - so both ends of that bar have an owner, and the head has
        // a pillar to be grouped with.
        const long own = recipe.add_pillar(head.id, headjp.z() - bridgestart.z());
        const size_t first_junction = recipe.junctioncount();
        recipe.add_junction(bridgestart, r);
        recipe.own_junctions_from(first_junction, own);
        recipe.add_bridge_between(nearpillar_id, bridgestart, bridgeend, r,
                                  BridgeKind::head, own);
    } else {
        recipe.add_bridge_to_pillar(head.id, bridgeend, nearpillar_id);
    }

    recipe.increment_bridges(nearpillar_id);

    return plan;
}

std::optional<RoutePlan> DefaultSupportTree::plan_ground_pillar(const Junction &hjp,
                                                                const Vec3d    &sourcedir,
                                                                long            head_id) const
{
    std::optional<PillarRecipe> planned = sla::plan_ground_pillar(suptree_ex_policy,
                                                                  m_sm,
                                                                  hjp.pos,
                                                                  sourcedir,
                                                                  hjp.r,
                                                                  hjp.r,
                                                                  head_id);
    if (!planned)
        return {};

    return pillar_route(RoutePlan::Kind::own_pillar, std::move(*planned));
}

long DefaultSupportTree::create_center_pillar(const Head &head)
{
    std::optional<RoutePlan> plan = plan_ground_pillar(head.junction(), head.dir, head.id);
    if (!plan)
        return SupportTreeNode::ID_UNSET;

    // Judged before it goes in, against the heads and what the centres before
    // it made; where it is built does not depend on the outcome.
    plan->prims = route_prims(plan->recipe, head);
    const auto pinhead = m_pinhead_prims.find(head.id);
    assert(pinhead != m_pinhead_prims.end());
    const std::vector<clearance::Contact> contacts =
        centroid_contacts(plan->prims, m_clearance.prims()[pinhead->second], m_clearance,
                          [this](long id) { return pillar_axis(id); },
                          m_clearance_report.native);

    const long pillar_id = commit(*plan, head.id);

    for (const clearance::Contact &ct : contacts) {
        m_clearance_report.centroid.push_back({pillar_id, ct});
        log_centroid_record(m_clearance_report.centroid.back());
    }

    return pillar_id;
}

RouteList DefaultSupportTree::route_list(const HeadRoutes &routes)
{
    RouteList list;
    list.reserve(routes.to_pillar.size() + routes.fallback.size());
    for (const RoutePlan &p : routes.to_pillar)
        list.push_back(&p);
    if (routes.fallback_shown)
        for (const RoutePlan &p : routes.fallback)
            list.push_back(&p);
    return list;
}

clearance::JointAxis DefaultSupportTree::pillar_axis(long pillar_id) const
{
    // Only pillars that are built: the ids of a plan not yet committed are
    // its own, and nothing committed shares them.
    assert(!BuildRecipe::is_local(pillar_id));
    const Pillar &p = m_builder.pillar(pillar_id);
    return {Vec2d{p.endpt.x(), p.endpt.y()}, std::max(p.r_start, p.r_end)};
}

long DefaultSupportTree::commit(const RoutePlan &plan, long head_id)
{
    const RecipeIds ids = replay(m_builder, plan.recipe);

    long pillar_id = SupportTreeNode::ID_UNSET;
    if (plan.new_pillar != SupportTreeNode::ID_UNSET) {
        // Save the pillar endpoint in the spatial index
        pillar_id = ids.pillar(plan.new_pillar);
        m_pillar_index.insert(m_builder.pillar(pillar_id).endpt, unsigned(pillar_id));
    }

    // What the route made goes into the clearance index, under the builder's
    // ids, and the head learns which pillar carries it: the one it sits on,
    // else the one its bar reaches (as the support-tree export names it).
    std::vector<clearance::Prim> prims = plan.prims;
    to_builder_ids(prims, ids);
    for (const clearance::Prim &p : prims)
        m_clearance.insert(p);

    const long sits_on = std::as_const(m_builder).head(unsigned(head_id)).pillar_id;
    const long carrier = sits_on >= 0 ? sits_on :
                         plan.kind == RoutePlan::Kind::to_pillar ? plan.target_pillar :
                                                                   pillar_id;
    if (const auto it = m_pinhead_prims.find(head_id); it != m_pinhead_prims.end())
        m_clearance.set_carrier(it->second, carrier);

    if (plan.kind == RoutePlan::Kind::to_pillar)
        note_prior_attachment(plan.target_pillar);

    m_penetration_stats.ray_misses += plan.ray_misses;
    if (plan.negative_anchor_width)
        BOOST_LOG_TRIVIAL(warning) << "Pinhead width is negative!";

    return pillar_id;
}

void DefaultSupportTree::add_pinheads()
{
    // The minimum distance for two support points to remain valid.
    const double /*constexpr*/ D_SP = 0.1;

    // Get the points that are too close to each other and keep only the
    // first one
    auto aliases = cluster(m_points, D_SP, 2);

    PtIndices filtered_indices;
    filtered_indices.reserve(aliases.size());
    m_iheads.reserve(aliases.size());
    for(auto& a : aliases) {
        // Here we keep only the front point of the cluster.
        filtered_indices.emplace_back(a.front());
    }

    // calculate the normals to the triangles for filtered points
    auto nmls = normals(suptree_ex_policy, m_points, m_sm.emesh,
                        m_sm.cfg.head_front_radius_mm, m_thr,
                        filtered_indices);

    // Not all of the support points have to be a valid position for
    // support creation. The angle may be inappropriate or there may
    // not be enough space for the pinhead. Filtering is applied for
    // these reasons.

    auto heads = reserve_vector<Head>(m_sm.pts.size());
    for (const SupportPoint &sp : m_sm.pts) {
        m_thr();
        heads.emplace_back(
            NaNd,
            sp.head_front_radius,
            0.,
            point_head_penetration_mm(sp, m_sm.cfg.head_penetration_mm),
            Vec3d::Zero(),         // dir
            sp.pos.cast<double>()  // displacement
            );
    }

    // Pinhead-angle optimizer budget. The default is the validated fast config:
    // stop each genetic search the instant it clears the required distance w
    // (the same gate used to accept the head, mirroring the branching tree's
    // optimize_pinhead_placement()), capped at 100 iterations. This cuts the
    // pinhead phase by ~17x (the optimizer dominated it) with no measurable
    // change to the supports. Env vars still override for A/B tuning:
    //   SLA_GA_MAX_ITER, SLA_GA_REL_SCORE, SLA_GA_STOP_SCORE (0 to disable).
    static const unsigned GA_DEFAULT_MAX_ITER = 100;
    const char *env_iter = std::getenv("SLA_GA_MAX_ITER");
    const char *env_rel  = std::getenv("SLA_GA_REL_SCORE");
    const char *env_stop = std::getenv("SLA_GA_STOP_SCORE");
    const unsigned ga_max_iter  = env_iter ? unsigned(std::strtoul(env_iter, nullptr, 10))
                                           : GA_DEFAULT_MAX_ITER;
    const double   ga_rel_score = env_rel  ? std::strtod(env_rel, nullptr)
                                           : m_sm.cfg.optimizer_rel_score_diff;
    const bool ga_use_stop = env_stop ? (std::atoi(env_stop) != 0) : true;

    std::function<void(unsigned, size_t, double)> filterfn;
    filterfn = [this, &nmls, &heads, &filterfn, ga_max_iter, ga_rel_score, ga_use_stop](unsigned fidx, size_t i, double back_r) {
        m_thr();

        Vec3d n = nmls.row(Eigen::Index(i));

        // for all normals we generate the spherical coordinates and
        // saturate the polar angle to 45 degrees from the bottom then
        // convert back to standard coordinates to get the new normal.
        // Then we just create a quaternion from the two normals
        // (Quaternion::FromTwoVectors) and apply the rotation to the
        // arrow head.

        // Everything below reads this point's own sizes, falling back to the
        // global configuration wherever the point leaves a field unset. Note
        // back_r is a parameter rather than a lookup: filterfn() re-enters
        // itself with the global fallback radius when the resolved one leaves
        // no room for a head.
        const SupportPoint &sp    = m_sm.pts[fidx];
        const double pt_back_r    = point_head_back_radius_mm(sp, m_sm.cfg.head_back_radius_mm);
        const double pt_width     = point_head_width_mm(sp, m_sm.cfg.head_width_mm);
        const double pt_penetr    = point_head_penetration_mm(sp, m_sm.cfg.head_penetration_mm);
        const double pt_slope     = point_bracing_angle_rad(sp, m_sm.cfg.bridge_slope);

        // A caller that placed this support by hand may have said which way its
        // head points. Dragging one moves the PILLAR while the tip stays in the
        // model, so the head leans and stretches to bridge the gap - somewhere
        // the search below, which only ever looks around the surface normal,
        // would never go. Taken as given: the caller is choosing, and the checks
        // that follow exist to second-guess a normal, not a decision.
        if (point_has_head_dir(m_sm.pts[fidx])) {
            const SupportPoint &sp0 = m_sm.pts[fidx];
            // Filled in the same way the search below fills it, on the head
            // already built from this point's position and penetration.
            Head &h = heads[fidx];
            h.id        = fidx;
            h.dir       = sp0.head_dir.cast<double>().normalized();
            h.width_mm  = point_head_width_mm(sp0, m_sm.cfg.head_width_mm);
            h.r_back_mm = point_head_back_radius_mm(sp0, m_sm.cfg.head_back_radius_mm);
            return;
        }

        auto [polar, azimuth] = dir_to_spheric(n);

        // skip if the tilt is not sane
        if (polar < PI - m_sm.cfg.normal_cutoff_angle) return;

        // NO overhang angle gate here, on purpose. It runs once, upstream, in
        // Phase 3 of support_points() (step slaposSupportPoints), so that an
        // exported point list no longer carries points this step would have
        // dropped for angle, and so that an IMPORTED list is never re-filtered.
        // Angle is the only dimension made to agree that way - a point can
        // still lose its head below to collision or to cluster dedup.
        // Re-adding a passes_overhang_filter() call here brings back both the
        // orphan points and the silent re-filtering of the user's own points
        // (capability sla-overhang-threshold-semantics).

        // We saturate the polar angle to 3pi/4
        polar = std::max(polar, PI - pt_slope);

        // save the head (pinpoint) position
        Vec3d hp = m_points.row(fidx);

        double lmin = pt_width, lmax = lmin;

        // back_r below this point's own radius means filterfn() re-entered with
        // the global fallback radius, i.e. this is a mini pillar.
        if (back_r < pt_back_r) {
            lmin = 0., lmax = pt_penetr;
        }

        // The distance needed for a pinhead to not collide with model.
        double w = lmin + 2 * back_r + 2 * m_sm.cfg.head_front_radius_mm -
                   pt_penetr;

        double pin_r = double(m_sm.pts[fidx].head_front_radius);

        // Reassemble the now corrected normal
        auto nn = spheric_to_dir(polar, azimuth).normalized();

        // check available distance
        AABBMesh::hit_result t = pinhead_mesh_intersect(hp, nn, pin_r, back_r, w);

        if (t.distance() < w) {
            // Let's try to optimize this angle, there might be a
            // viable normal that doesn't collide with the model
            // geometry and its very close to the default.

            auto crit = get_criteria(m_sm.cfg).max_iterations(ga_max_iter)
                                              .rel_score_diff(ga_rel_score);
            if (ga_use_stop)
                crit.stop_score(w); // stop as soon as a non-colliding angle is found
            Optimizer<AlgNLoptGenetic> solver(crit);
            solver.seed(0); // we want deterministic behavior

            auto oresult = solver.to_max().optimize(
                [this, pin_r, back_r, hp](const opt::Input<3> &input)
                {
                    auto &[plr, azm, l] = input;

                    auto dir = spheric_to_dir(plr, azm).normalized();

                    return pinhead_mesh_intersect(
                               hp, dir, pin_r, back_r, l).distance();
                },
                initvals({polar, azimuth, (lmin + lmax) / 2.}), // start with what we have
                bounds({
                    {PI - pt_slope, PI},    // Must not exceed the slope limit
                    {-PI, PI}, // azimuth can be a full search
                    {lmin, lmax}
                }));

            if(oresult.score > w) {
                polar = std::get<0>(oresult.optimum);
                azimuth = std::get<1>(oresult.optimum);
                nn = spheric_to_dir(polar, azimuth).normalized();
                lmin = std::get<2>(oresult.optimum);
                t = AABBMesh::hit_result(oresult.score);
            }
        }

        if (t.distance() > w && hp.z() + w * nn.z() >= ground_level(m_sm)) {
            Head &h = heads[fidx];
            h.id        = fidx;
            h.dir       = nn;
            h.width_mm  = lmin;
            h.r_back_mm = back_r;

            // Thin wall protection, applied here and nowhere earlier. The angle
            // search above - including the optimizer objective - runs entirely
            // on the configured penetration, so the direction is picked exactly
            // as it was before; only the committed head is trimmed. Measuring
            // inside the objective would make the clamp depend on the direction
            // while the direction is being scored through the clamp.
            //
            // This has to sit before the junction and pillar phases rather than
            // after: junction() is pos + (fullwidth() - r_back) * dir and
            // fullwidth() is real_width() - penetration, so shrinking the
            // penetration pushes the junction outwards. Doing it later would
            // leave the pillar starting at the old junction while the head mesh
            // ends at the new one - a gap of up to the configured penetration.
            h.penetration_mm = clamped_head_penetration(m_sm.emesh, hp, nn,
                                                        pt_penetr,
                                                        &m_penetration_stats);
        } else if (back_r > m_sm.cfg.head_fallback_radius_mm) {
            // The fallback radius stays global: it is the "no room for a real
            // head, try a mini pillar" escape hatch, not one of the point's
            // own sizes.
            filterfn(fidx, i, m_sm.cfg.head_fallback_radius_mm);
        }
    };

    execution::for_each(
        suptree_ex_policy, size_t(0), filtered_indices.size(),
        [this, &filterfn, &filtered_indices](size_t i) {
            const unsigned fidx = filtered_indices[i];
            filterfn(fidx, i,
                     point_head_back_radius_mm(m_sm.pts[fidx],
                                               m_sm.cfg.head_back_radius_mm));
        },
        execution::max_concurrency(suptree_ex_policy));

    for (size_t i = 0; i < heads.size(); ++i)
        if (heads[i].is_valid()) {
            m_builder.add_head(i, heads[i]);
            m_iheads.emplace_back(i);
        }

    m_thr();
}

void DefaultSupportTree::classify()
{
    // We should first get the heads that reach the ground directly
    PtIndices ground_head_indices;
    ground_head_indices.reserve(m_iheads.size());
    m_iheads_onmodel.reserve(m_iheads.size());

    // First we decide which heads reach the ground and can be full
    // pillars and which shall be connected to the model surface (or
    // search a suitable path around the surface that leads to the
    // ground -- TODO)
    for(unsigned i : m_iheads) {
        m_thr();

        Head &head = m_builder.head(i);
        double r = head.r_back_mm;
        Vec3d headjp = head.junction_point();

           // collision check
        auto hit = bridge_mesh_intersect(headjp, DOWN, r);

        if(std::isinf(hit.distance())) ground_head_indices.emplace_back(i);
        else if(m_sm.cfg.ground_facing_only)  head.invalidate();
        else m_iheads_onmodel.emplace_back(i);

        m_head_to_ground_scans[i] = hit;
    }

    // We want to search for clusters of points that are far enough
    // from each other in the XY plane to not cross their pillar bases
    // These clusters of support points will join in one pillar,
    // possibly in their centroid support point.

    auto pointfn = [this](unsigned i) {
        return m_builder.head(i).junction_point();
    };

    auto predicate = [this](const PointIndexEl &e1,
                            const PointIndexEl &e2) {
        double d2d = distance(to_2d(e1.first), to_2d(e2.first));
        double d3d = distance(e1.first, e2.first);
        // The 2D test asks "would these two pillar bases overlap", so it has to
        // add up the two bases actually being built, not twice the global one.
        // .second is the head id, which is the support point index. With every
        // point on the default this is exactly 2 * base_radius_mm as before.
        double bases = resolved_base_radius_mm(m_sm, long(e1.second)) +
                       resolved_base_radius_mm(m_sm, long(e2.second));
        return d2d < bases && d3d < m_sm.cfg.max_bridge_length_mm;
    };

    m_pillar_clusters = cluster(ground_head_indices, pointfn, predicate,
                                m_sm.cfg.max_bridges_on_pillar);

    // Every head that is left is in the way of every route from here on,
    // routed or not. Which pillar carries it is named once it is routed.
    for (const Head &head : m_builder.heads()) {
        if (!head.is_valid())
            continue;
        clearance::Prim p;
        p.cap     = clearance::pinhead_capsule(head.pos, head.junction_point(), head.r_back_mm);
        p.kind    = clearance::PrimKind::pinhead;
        p.head_id = head.id;
        m_pinhead_prims[head.id] = m_clearance.insert(p);
    }
}

void DefaultSupportTree::routing_to_ground()
{
    std::vector<RouteJob> jobs;

    for (auto &cl : m_pillar_clusters) {
        m_thr();

        // place all the centroid head positions into the index. We
        // will query for alternative pillar positions. If a sidehead
        // cannot connect to the cluster centroid, we have to search
        // for another head with a full pillar. Also when there are two
        // elements in the cluster, the centroid is arbitrary and the
        // sidehead is allowed to connect to a nearby pillar to
        // increase structural stability.

        if (cl.empty()) continue;

           // get the current cluster centroid
        auto &      thr    = m_thr;
        const auto &points = m_points;

        long lcid = cluster_centroid(
            cl, [&points](size_t idx) { return points.row(long(idx)); },
            [thr](const Vec3d &p1, const Vec3d &p2) {
                thr();
                return distance(Vec2d(p1.x(), p1.y()), Vec2d(p2.x(), p2.y()));
            });

        assert(lcid >= 0);
        unsigned hid = cl[size_t(lcid)]; // Head ID

        const Head &h = std::as_const(m_builder).head(hid);

        // The centre pillar is kept as it is made. It used to be looked up
        // later as the pillar nearest the centre head, which is some other
        // pillar whenever this one could not be made.
        const long center = create_center_pillar(h);
        if (center < 0) {
            BOOST_LOG_TRIVIAL(warning)
                << "Pillar cannot be created for support point id: " << hid;
            m_iheads_onmodel.emplace_back(h.id);
        }

        // The other heads of the cluster go to the centre pillar, else to
        // another pillar nearby, else down on a pillar of their own.
        for (unsigned c : cl)
            if (c != hid)
                jobs.push_back({c, center});
    }

    const RouteCounts counts = route_heads(RouteStep::ground, std::move(jobs));
    BOOST_LOG_TRIVIAL(info) << "Routing to ground: " << counts.routed << " heads routed, "
                            << counts.recomputed << " planned again, " << counts.unrouted
                            << " without a route";
}

namespace {

std::optional<RoutePlan> ground_route(const Head &head, std::optional<PillarRecipe> &&planned)
{
    if (!planned)
        return {};

    planned->recipe.set_head_pillar(head.id, planned->pillar);
    return pillar_route(RoutePlan::Kind::ground_route, std::move(*planned));
}

} // namespace

std::optional<RoutePlan> DefaultSupportTree::plan_ground_route(const Head &head) const
{
    return ground_route(head, sla::plan_connect_to_ground(suptree_ex_policy, m_sm,
                                                          {head.junction_point(),
                                                           head.r_back_mm},
                                                          head.dir,
                                                          head.r_back_mm,
                                                          head.id));
}

std::optional<RoutePlan> DefaultSupportTree::plan_ground_detour(const Head &head) const
{
    return ground_route(head, sla::plan_ground_detour(suptree_ex_policy, m_sm,
                                                      {head.junction_point(),
                                                       head.r_back_mm},
                                                      head.r_back_mm,
                                                      head.dir,
                                                      head.id));
}

std::optional<RoutePlan> DefaultSupportTree::plan_model_anchor(const Head &head) const
{
    if (head.id <= SupportTreeNode::ID_UNSET) return {};

    auto it = m_head_to_ground_scans.find(unsigned(head.id));
    if (it == m_head_to_ground_scans.end()) return {};

    auto &hit = it->second;

    if (!hit.is_hit()) {
        // TODO scan for potential anchor points on model surface
        return {};
    }

    Vec3d hjp = head.junction_point();
    double zangle = std::asin(hit.direction().z());
    zangle = std::max(zangle, PI/4);
    double h = std::sin(zangle) * head.fullwidth();

    // The width of the tail head that we would like to have...
    h = std::min(hit.distance() - head.r_back_mm, h);

    // If this is a mini pillar dont bother with the tail width, can be 0.
    // "Mini" is relative to the radius this point asked for, not the global one.
    if (head.r_back_mm < resolved_head_back_radius_mm(m_sm, head.id))
        h = std::max(h, 0.);
    else if (h <= 0.) return {};

    Vec3d endp{hjp.x(), hjp.y(), hjp.z() - hit.distance() + h};
    auto center_hit = m_sm.emesh.query_ray_hit(hjp, DOWN);

    double hitdiff = center_hit.distance() - hit.distance();
    Vec3d hitp = std::abs(hitdiff) < 2*head.r_back_mm?
                     center_hit.position() : hit.position();

    RoutePlan plan;
    plan.kind       = RoutePlan::Kind::model_anchor;
    plan.new_pillar = plan.recipe.add_pillar(head.id, hjp.z() - endp.z());

    // Direction of the anchor pin, pointing from the model surface up towards
    // the pillar end. Head stores this vector verbatim and multiplies by it in
    // junction(): pos + (fullwidth() - r_back_mm) * dir. A non-unit dir therefore
    // stretches the junction by |dir| - and |endp - hitp| here is the anchor's
    // own length in millimetres, not 1, so the junction lands far past where it
    // belongs. Upstream 2.9.6 normalizes at this exact spot (see PrusaSlicer
    // DefaultSupportTree.cpp:706); this fork predates that fix. #5 also needs a
    // unit dir to cast its wall thickness ray, which is why this has to land
    // before it. Note the length is not taken from this vector: `dist` below
    // recomputes it from the same two points, so normalizing changes nothing else.
    //
    // The zero-length guard is ours, not upstream's. endp and hitp coincide when
    // a mini pillar's tail width collapses to zero on a straight down scan, and
    // Eigen's normalized() turns that into a NaN direction silently, which would
    // then flow into the anchor mesh. The old code degenerated to a zero vector
    // instead - equally meaningless, but at least finite. Fall back to straight
    // up, the direction this vector provably has whenever it is not degenerate
    // (endp.z() - hitp.z() == h >= 0 with the x/y components equal).
    const Vec3d taildiff = endp - hitp;
    const Vec3d taildir  = taildiff.squaredNorm() > EPSILON * EPSILON ?
                               Vec3d{taildiff.normalized()} : Vec3d{-DOWN};
    // Thin wall protection for the anchor, mirroring what add_pinheads() does
    // for the main heads. An anchor is a reverse pinhead: its pin travels along
    // the head's local +z, which maps to -taildir, and taildir points from the
    // model surface up towards the pillar end - so -taildir goes down into the
    // model body and hitp/taildir are exactly the contact point and head axis
    // the measurement wants.
    //
    // There is no angle search to protect here (taildir falls straight out of
    // the geometry, no optimizer is involved), so unlike the main pinhead the
    // clamp is applied before `dist` rather than after a search - and it has to
    // be, because `dist` and the `w` derived from it both carry the penetration.
    // Handing add_anchor() a clamped penetration together with a `w` sized for
    // the configured one would leave the anchor's length disagreeing with its
    // bite by exactly the amount that was trimmed.
    //
    // A miss is counted with the plan and added on commit: planning writes
    // nothing shared, and an anchor never committed has nothing to report.
    PenetrationClampStats stats;
    const double anchor_penetration =
        clamped_head_penetration(m_sm.emesh, hitp, taildir,
                                 resolved_head_penetration_mm(m_sm, head.id),
                                 &stats);
    plan.ray_misses = stats.ray_misses.load();

    double dist = (hitp - endp).norm() + anchor_penetration;
    double w = dist - 2 * head.r_pin_mm - head.r_back_mm;

    if (w < 0.) {
        plan.negative_anchor_width = true; // "Pinhead width is negative!" on commit
        w = 0.;
    }

    plan.recipe.add_anchor(head.r_back_mm, head.r_pin_mm, w,
                           anchor_penetration, taildir, hitp);

    return plan;
}

void DefaultSupportTree::plan_near_pillars(const RouteView        &view,
                                           const Head             &head,
                                           long                    skip,
                                           size_t                  limit,
                                           std::vector<RoutePlan> &out) const
{
    // Pillars by the distance of their feet from the ground under the
    // junction, as the nearest neighbour search always took it.
    const Vec3d jp = head.junction_point();
    const Vec3d qp(jp.x(), jp.y(), ground_level(m_sm));

    // Only the pillars within the head's reach in plan view: plan_to_pillar()
    // turns the others down on the same test. A pillar's foot is under its top.
    const double reach = bridge_reach(head);

    std::vector<NearPillar> near;
    near.reserve(view.feet.size());
    const size_t pillarcount = view.builder.pillarcount();
    for (const PointIndexEl &el : view.feet) {
        const long id = long(el.second);
        if (id != skip && size_t(id) < pillarcount &&
            !(distance(to_2d(jp), to_2d(el.first)) > reach))
            near.push_back({(el.first - qp).squaredNorm(), id});
    }

    // Sorted by nearer(), a total order: the same pillars come in the same
    // order, however the index lists them. A growing slice at a time: most
    // heads find their pillars among the first few.
    size_t next = 0;
    for (size_t slice = 16; out.size() < limit && next < near.size(); slice *= 2) {
        const size_t upto = std::min(near.size(), next + slice);
        std::partial_sort(near.begin() + next, near.begin() + upto, near.end(), nearer);

        for (; next < upto && out.size() < limit; ++next) {
            m_thr();
            if (std::optional<RoutePlan> p = plan_to_pillar(view.builder, head, near[next].id))
                out.emplace_back(std::move(*p));
        }
    }
}

void DefaultSupportTree::plan_routes(RouteStep        step,
                                     const RouteView &view,
                                     const Head      &head,
                                     long             center,
                                     bool             recompute,
                                     HeadRoutes      &routes) const
{
    routes.to_pillar.clear();
    if (center >= 0)
        if (std::optional<RoutePlan> p = plan_to_pillar(view.builder, head, center))
            routes.to_pillar.emplace_back(std::move(*p));
    plan_near_pillars(view, head, center, ROUTE_PILLAR_CANDIDATES, routes.to_pillar);
    for (RoutePlan &p : routes.to_pillar)
        p.prims = route_prims(p.recipe, head);

    // The fallbacks cost the most, and the old flow only got to them with no
    // pillar in reach.
    if (!recompute && !routes.to_pillar.empty())
        return;
    routes.fallback_shown = true;
    if (!routes.fallback_planned)
        plan_fallbacks(step, head, routes);
}

void DefaultSupportTree::plan_fallbacks(RouteStep step, const Head &head, HeadRoutes &routes) const
{
    routes.fallback_planned = true;

    auto add = [&routes, &head](std::optional<RoutePlan> &&p) {
        if (!p) return;
        p->prims = route_prims(p->recipe, head);
        routes.fallback.emplace_back(std::move(*p));
    };

    if (step == RouteStep::ground) {
        add(plan_ground_pillar(head.junction(), head.dir, head.id));
    } else {
        std::optional<RoutePlan> route = plan_ground_route(head);
        if (!route)
            route = plan_ground_detour(head);
        add(std::move(route));
        // No route to the ground, so connect to the model body as a last resort
        add(plan_model_anchor(head));
    }
}

DefaultSupportTree::RouteCounts DefaultSupportTree::route_heads(RouteStep             step,
                                                                std::vector<RouteJob> jobs)
{
    std::sort(jobs.begin(), jobs.end(),
              [](const RouteJob &a, const RouteJob &b) { return a.head < b.head; });

    // Phase one: every head at once, against the state the step starts from.
    // Each writes its own slot and nothing else. Each is planned in isolation:
    // a thread waiting for the parallel mesh tests inside a detour search then
    // takes no other head's planning meanwhile, whose own search would draw
    // from the same per thread random state (see sla::plan_ground_detour()).
    //
    // A head whose every pillar candidate is too close already to what is
    // built when the step starts is too close at commit too, but for the
    // rare joint a head's carrier clips away later: what is built only grows.
    // It is bound to plan again on one thread, so its fallbacks -
    // the dear part, a detour search among them - are planned here, in
    // parallel. They stay hidden until that recompute shows them, so which
    // route is chosen does not change.
    const clearance::PillarAxisFn axis_of = [this](long id) { return pillar_axis(id); };
    std::vector<HeadRoutes> routes(jobs.size());
    {
        const RouteView view = route_view();
        auto too_close = [this, &axis_of](const RoutePlan &p) {
            return !m_clearance.violations(p.prims, clearance::SAFETY_CLEARANCE, axis_of).empty();
        };
        execution::for_each(
            suptree_ex_policy, size_t(0), jobs.size(),
            [this, step, &view, &jobs, &routes, &too_close](size_t i) {
                m_thr();
                tbb::this_task_arena::isolate([&] {
                    const Head &head = view.builder.head(jobs[i].head);
                    HeadRoutes &r    = routes[i];
                    plan_routes(step, view, head, jobs[i].center, false, r);
                    if (!r.fallback_planned &&
                        std::all_of(r.to_pillar.begin(), r.to_pillar.end(), too_close))
                        plan_fallbacks(step, head, r);
                });
            },
            execution::max_concurrency(suptree_ex_policy));
    }

    // Phase two: one head after the other, smallest id first, each committing
    // one route whole, the one select_route() picks on what is built now. A
    // head none of whose candidates keeps the clearance plans once more,
    // against what is there now - the pillars the heads before it made
    // included - and with its fallbacks shown, planned now unless phase one
    // planned them already.
    RouteCounts counts;
    for (size_t i = 0; i < jobs.size(); ++i) {
        m_thr();

        const Head &head = std::as_const(m_builder).head(jobs[i].head);

        // The rules no level relaxes (design.md decision 8). The bridge cap
        // is asked here, on the counts the routes before this one left.
        auto hard_ok = [this, &head](const RoutePlan &p) {
            return hard_rules_ok(m_builder, p, head, m_sm.cfg.max_bridges_on_pillar);
        };
        auto recompute = [&, i] {
            plan_routes(step, route_view(), head, jobs[i].center, true, routes[i]);
            return route_list(routes[i]);
        };

        const RouteChoice choice = select_route(route_list(routes[i]), m_clearance, axis_of,
                                                hard_ok, recompute);
        counts.recomputed += choice.recomputed;

        if (choice.plan) {
            commit(*choice.plan, head.id);
            ++counts.routed;

            if (choice.level != RouteLevel::L0) {
                ++(choice.level == RouteLevel::L1 ? m_clearance_report.l1 :
                                                    m_clearance_report.l2);
                m_clearance_report.routes.push_back(
                    {head.id, choice.plan->kind, choice.level, choice.contacts});
                log_route_record(m_clearance_report.routes.back());
            }
            continue;
        }

        ++counts.unrouted;
        ++m_clearance_report.none;

        // A ground facing head with no route is left as it was, a model
        // facing one is dropped: both as they always were.
        if (step == RouteStep::model) {
            BOOST_LOG_TRIVIAL(warning)
                << "Failed to route model facing support point. ID: " << jobs[i].head;

            m_builder.head(jobs[i].head).invalidate();
        }
    }

    return counts;
}

void DefaultSupportTree::routing_to_model()
{
    // We need to check if there is an easy way out to the bed surface.
    // If it can be routed there with a bridge shorter than
    // min_bridge_distance.
    //
    // Each head: a nearby pillar, else a route to the ground, else an anchor
    // on the model body.
    std::vector<RouteJob> jobs;
    jobs.reserve(m_iheads_onmodel.size());
    for (unsigned idx : m_iheads_onmodel)
        jobs.push_back({idx, SupportTreeNode::ID_UNSET});

    const RouteCounts counts = route_heads(RouteStep::model, std::move(jobs));
    BOOST_LOG_TRIVIAL(info) << "Routing to model: " << counts.routed << " heads routed, "
                            << counts.recomputed << " planned again, " << counts.unrouted
                            << " without a route";
}

void DefaultSupportTree::adopt_prior_pillars()
{
    m_prior_pillar_ids.clear();
    m_prior_pillar_ids.reserve(m_sm.prior.size());

    for (const PriorPillar &pp : m_sm.prior) {
        long pid = m_builder.add_frozen_pillar(pp.endpoint, pp.height,
                                               pp.r_start, pp.r_end,
                                               pp.links, pp.bridges, pp.pairs);
        m_pillar_index.insert(m_builder.pillar(pid).endpoint(), unsigned(pid));

        // Only the pillar: the bars it came with are not handed in.
        clearance::Prim p;
        p.cap      = clearance::pillar_capsule(pp.endpoint, pp.height, pp.r_start, pp.r_end);
        p.kind     = clearance::PrimKind::pillar;
        p.pillar_a = pid;
        m_clearance.insert(p);

        m_prior_pillar_ids.push_back(pid);
        m_prior_caller_ids[pid] = pp.id;
    }
}

void DefaultSupportTree::note_prior_attachment(long pillar_id)
{
    if (pillar_id < 0 || size_t(pillar_id) >= m_builder.pillarcount())
        return;
    if (m_builder.pillar(pillar_id).frozen)
        m_attached_prior_ids.insert(pillar_id);
}

PriorAttachments DefaultSupportTree::prior_attachments() const
{
    PriorAttachments out;
    out.reserve(m_attached_prior_ids.size());
    for (long pid : m_attached_prior_ids) {
        const auto it = m_prior_caller_ids.find(pid);
        if (it == m_prior_caller_ids.end())
            continue;
        const Pillar &p = m_builder.pillar(pid);
        out.push_back(PriorAttachment{it->second, p.links, p.bridges, p.pairs});
    }
    return out;
}

void DefaultSupportTree::interconnect_pillars()
{
    // Now comes the algorithm that connects pillars with each other.
    // Ideally every pillar should be connected with at least one of its
    // neighbors if that neighbor is within max_pillar_link_distance.
    //
    // Which pairs get braced is decided by the BracingPlanner: every candidate
    // pair at once, shortest first, so a near neighbour is braced before a far
    // one can reach over it; no two groups may cross or crowd each other in
    // plan view, none may pass a third pillar, and every bar keeps its distance
    // from the bars already there. See BracingPlanner.hpp and openspec change
    // sla-bracing-planar-topology.

    // Pillars with height exceeding H1 will require at least one neighbor
    // to connect with. Height exceeding H2 require two neighbors.
    double H1 = m_sm.cfg.max_solo_pillar_height_mm;
    double H2 = m_sm.cfg.max_dual_pillar_height_mm;

    using namespace bracing;

    // The planner is pure geometry and does not see SupportTreeConfig; the pair
    // cap it was given must still be the count the legacy cascade used.
    static_assert(PAIR_CAP == SupportTreeConfig::pillar_cascade_neighbors,
                  "PAIR_CAP must match legacy cascade neighbor count");

    PlannerConfig pc;
    pc.solo_height         = H1;
    pc.max_link_distance   = m_sm.cfg.max_pillar_link_distance_mm;
    pc.min_pillar_distance = 2 * m_sm.cfg.head_back_radius_mm;
    pc.truss.bridge_slope_rad = m_sm.cfg.bridge_slope;
    pc.truss.ground_z         = ground_level(m_sm);
    pc.truss.base_height      = m_sm.cfg.base_height_mm;
    pc.truss.base_radius      = m_sm.cfg.base_radius_mm;
    pc.truss.mode             = m_sm.cfg.pillar_connection_mode;

    // The model test interconnect() always made on a bar: clear for the bar's
    // whole length, which is the pillar distance over cos(slope).
    const double cos_slope = std::cos(-m_sm.cfg.bridge_slope);
    BracingPlanner planner(pc, [this, cos_slope](const Vec3d &from, const Vec3d &to, double r) {
        const double len = (Vec2d{to.x(), to.y()} - Vec2d{from.x(), from.y()}).norm() / cos_slope;
        return bridge_mesh_distance(from, dirv(from, to), r) >= len;
    });

    // The pillars in the index are the ones bracing has always worked with.
    // The rest (a head's own short pillar under a bar to a neighbour) are
    // never braced, but stand in the way like every other pillar.
    std::set<long> braceable;
    m_pillar_index.foreach([&braceable](const PointIndexEl &el) {
        braceable.insert(long(el.second));
    });

    auto geom_of = [](const Pillar &p, bool can_brace) {
        PillarGeom g;
        g.id        = p.id;
        g.xy        = {p.endpt.x(), p.endpt.y()};
        g.foot_z    = p.endpt.z();
        g.height    = p.height;
        g.r         = p.r_start;
        g.links     = p.links;
        g.pairs     = p.pairs;
        g.frozen    = p.frozen;
        g.braceable = can_brace;
        return g;
    };

    for (const Pillar &p : m_builder.pillars())
        planner.add_pillar(geom_of(p, braceable.count(p.id) > 0));

    // Every bar that is already there - pins bridged to pillars, routes around
    // the model - is in the way of the bracing, bar by bar in 3D.
    for (const Bridge &b : m_builder.bridges())
        planner.add_obstacle_bar({{b.startp, b.endp, b.r}, b.owner_a, b.owner_b});
    for (const DiffBridge &b : m_builder.diffbridges())
        planner.add_obstacle_bar({{b.startp, b.endp, std::max(b.r, b.end_r)},
                                  b.owner_a, b.owner_b});
    for (const Bridge &b : m_builder.crossbridges())
        planner.add_truss_bar(b.owner_a, b.owner_b, {b.startp, b.endp, b.r});

    // The planner keeps the counts as it decides; the builder follows it.
    auto sync_counts = [this, &planner](long id) {
        const PillarGeom &g = planner.pillar(id);
        const Pillar     &p = m_builder.pillar(id);
        while (p.links < g.links) m_builder.increment_links(p);
        while (p.pairs < g.pairs) m_builder.increment_pairs(p);
    };

    auto lay = [this, &planner, &sync_counts](const Decision &d) {
        if (d.reason != Reason::accepted) return false;
        interconnect(planner.built()[size_t(d.truss)]);
        sync_counts(d.edge.a);
        sync_counts(d.edge.b);
        // Bracing to a pillar carried in from an earlier generation is the
        // whole point of additive support: report it, and what it did to that
        // pillar's counts.
        note_prior_attachment(d.edge.a);
        note_prior_attachment(d.edge.b);
        return true;
    };

    for (const Decision &d : planner.run_greedy()) lay(d);

    // A tall pillar left with no link gets one more try, with the bars judged
    // one by one in 3D instead of the plan rule between groups - before any
    // prop is grown for it.
    for (const Decision &d : planner.run_rescue()) lay(d);

    // We would be done here if we could allow some pillars to not be
    // connected with any neighbors. But this might leave the support tree
    // unprintable.
    //
    // The current solution is to insert additional pillars next to these
    // lonely pillars. One or even two additional pillar might get inserted
    // depending on the length of the lonely pillar.

    // Last of all, a tall pillar still without a link is braced to the
    // nearest pillar that gives it one, even past the pair cap - whether or
    // not props were grown.
    auto safety_net = [&planner, &lay]() {
        for (const Decision &d : planner.run_fallback()) lay(d);
    };

    // Propping a lonely pillar is the engine deciding the user needs more
    // supports than they asked for. Where placement is the user's, it is not.
    if (!m_sm.cfg.auxiliary_pillars) {
        safety_net();
        return;
    }

    size_t pillarcount = m_builder.pillarcount();

    // Again, go through all pillars, this time in the whole support tree
    // not just the index.
    for(size_t pid = 0; pid < pillarcount; pid++) {
        auto pillar = [this, pid]() { return m_builder.pillar(pid); };

        // Never prop up a frozen pillar: it was already resolved when it was
        // generated, and adding to it now would change existing geometry.
        if (pillar().frozen) continue;

        // Decide how many additional pillars will be needed:

        unsigned needpillars = 0;
        if (pillar().bridges > m_sm.cfg.max_bridges_on_pillar)
            needpillars = 3;
        else if (pillar().links < 2 && pillar().height > H2) {
            // Not enough neighbors to support this pillar
            needpillars = 2;
        } else if (pillar().links < 1 && pillar().height > H1) {
            // No neighbors could be found and the pillar is too long.
            needpillars = 1;
        }

        needpillars = std::max(pillar().links, needpillars) - pillar().links;
        if (needpillars == 0) continue;

        // Search for new pillar locations:

        bool   found    = false;
        double alpha    = 0; // goes to 2Pi
        double r        = 2 * m_sm.cfg.base_radius_mm;
        Vec3d  pillarsp = pillar().startpoint();

        // temp value for starting point detection
        Vec3d sp(pillarsp.x(), pillarsp.y(), pillarsp.z() - r);

        // A vector of bool for placement feasbility
        std::vector<bool>  canplace(needpillars, false);
        std::vector<Vec3d> spts(needpillars); // vector of starting points

        double gnd      = ground_level(m_sm);
        double min_dist = m_sm.cfg.pillar_base_safety_distance_mm +
                          m_sm.cfg.base_radius_mm + EPSILON;

        while(!found && alpha < 2*PI) {
            for (unsigned n = 0;
                 n < needpillars && (!n || canplace[n - 1]);
                 n++)
            {
                double a = alpha + n * PI / 3;
                Vec3d  s = sp;
                s.x() += std::cos(a) * r;
                s.y() += std::sin(a) * r;
                spts[n] = s;

                // Check the path vertically down
                Vec3d check_from = s + Vec3d{0., 0., pillar().r_start};
                auto hr = bridge_mesh_intersect(check_from, DOWN, pillar().r_start);
                Vec3d gndsp{s.x(), s.y(), gnd};

                // If the path is clear, check for pillar base collisions
                canplace[n] = std::isinf(hr.distance()) &&
                              std::sqrt(m_sm.emesh.squared_distance(gndsp)) >
                                  min_dist;

                // A place the prop could stand but not be braced from is no
                // place for it: the next angle may be both.
                if (canplace[n]) {
                    PillarGeom prop = geom_of(Pillar(gndsp, s.z() - gnd, pillar().r_start), true);
                    prop.id = long(m_builder.pillarcount() + n);
                    canplace[n] = planner.try_attach(pillar().id, prop).reason ==
                                  Reason::accepted;
                }
            }

            found = std::all_of(canplace.begin(), canplace.end(),
                                [](bool v) { return v; });

            // 20 angles will be tried...
            alpha += 0.1 * PI;
        }

        std::vector<long> newpills;
        newpills.reserve(needpillars);

        if (found)
            for (unsigned n = 0; n < needpillars; n++) {
                Vec3d s = spts[n];
                Pillar p(Vec3d{s.x(), s.y(), gnd}, s.z() - gnd, pillar().r_start);

                // Whether this prop can be braced to its pillar is asked
                // before it exists: only a prop that can is added. It is asked
                // under the id it will get, so the pair is the same one
                // attach() then builds.
                PillarGeom prop = geom_of(p, true);
                prop.id = long(m_builder.pillarcount());
                if (planner.try_attach(pillar().id, prop).reason != Reason::accepted)
                    continue;

                Pillar &pp = m_builder.pillar(m_builder.add_pillar(p));
                assert(pp.id == prop.id);
                planner.add_pillar(geom_of(pp, true));

                // An auxiliary pillar carries no head, so this is the only
                // thing that says which support it belongs to.
                pp.props_for = pillar().id;

                const Decision braced = planner.attach(pillar().id, pp.id);
                assert(braced.reason == Reason::accepted);
                lay(braced);

                add_pillar_base(pp.id);

                m_pillar_index.insert(pp.endpoint(), unsigned(pp.id));

                size_t first_junction = m_builder.junctioncount();
                m_builder.add_junction(s, pillar().r_start);
                m_builder.own_junctions_from(first_junction, pp.id);
                double t = bridge_mesh_distance(pillarsp, dirv(pillarsp, s),
                                                pillar().r_start);
                if (distance(pillarsp, s) < t) {
                    const Bridge prop_bar =
                        m_builder.add_bridge_between(pp.id, pillarsp, s,
                                                     pillar().r_start, BridgeKind::prop,
                                                     pillar().id);
                    // Later bracing has to keep clear of it like of any bar.
                    planner.add_obstacle_bar({{prop_bar.startp, prop_bar.endp, prop_bar.r},
                                              prop_bar.owner_a, prop_bar.owner_b});
                }

                if (pillar().endpoint().z() > ground_level(m_sm) + pillar().r_start) {
                    first_junction = m_builder.junctioncount();
                    m_builder.add_junction(pillar().endpoint(), pillar().r_start);
                    m_builder.own_junctions_from(first_junction, pillar().id);
                }

                newpills.emplace_back(pp.id);
            }

        if(!newpills.empty()) {
            for(auto it = newpills.begin(), nx = std::next(it);
                 nx != newpills.end(); ++it, ++nx)
                lay(planner.attach(*it, *nx));

            // Only the new props are run again: a frozen pillar never asks,
            // and the pillars that were here already have had their turn.
            for (const Decision &d : planner.run_greedy_for(newpills)) lay(d);
        }
    }

    safety_net();
}

}} // namespace Slic3r::sla

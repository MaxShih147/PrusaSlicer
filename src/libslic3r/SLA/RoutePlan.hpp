#ifndef SLA_ROUTEPLAN_HPP
#define SLA_ROUTEPLAN_HPP

// A head's route to the ground, planned before anything of it is built.
//
// Routing a head used to write into the builder as it went: a bar to a pillar,
// then the test that should have kept it out, then another bar. Here a route is
// planned first, as the list of builder calls that would make it (BuildRecipe),
// and written later in one go by replay(), or not at all. Planning reads the
// builder and never writes it, so all heads of a step can be planned at once
// against the same state, and committed one by one in head id order whatever
// the thread count (openspec change sla-head-bridge-clearance, design.md
// decision 5).

#include <cstddef>
#include <cstdint>
#include <vector>

#include <libslic3r/SLA/SupportTreeBuilder.hpp>

namespace Slic3r { namespace sla {

// One builder call of a recipe, with what replay() needs to make it again.
// Which fields an op reads is listed next to it; ids may be recipe-local.
struct BuildStep
{
    enum class Op : uint8_t {
        head_pillar,        // add_pillar(head, len)
        pillar,             // add_pillar(a, len, r, r2): foot, height, r_start, r_end
        pillar_base,        // add_pillar_base(pillar, len, r)
        junction,           // add_junction(a, r)
        own_junctions,      // own_junctions_from(<junction ref>, pillar)
        bridge,             // add_bridge(a, b, r, kind, pillar)
        bridge_between,     // add_bridge_between(pillar, a, b, r, kind, <pillar ref>)
        head_bridge,        // add_bridge_to_pillar(head, b, pillar)
        diffbridge,         // add_diffbridge(DiffBridge(a, b, r, r2))
        own_bridge,         // own_bridge(<bridge ref>, pillar)
        own_diffbridge,     // own_diffbridge(<diffbridge ref>, pillar)
        head_bridge_id,     // head(head).bridge_id = <bridge ref>
        head_diffbridge_id, // head(head).bridge_id = <diffbridge ref>
        head_pillar_id,     // head(head).pillar_id = pillar
        anchor,             // add_anchor(r, r2, len, pen, b, a): r_back, r_pin, width,
                            // penetration, direction, position
        increment_bridges,  // increment_bridges(pillar)
    };

    Op         op;
    long       head   = SupportTreeNode::ID_UNSET;
    long       pillar = SupportTreeNode::ID_UNSET;
    long       ref    = SupportTreeNode::ID_UNSET;
    BridgeKind kind   = BridgeKind::unset;
    Vec3d      a = Vec3d::Zero(), b = Vec3d::Zero();
    double     r = 0., r2 = 0., len = 0., pen = 0.;
};

// The builder calls of one route, in order. It answers the calls the routing
// code makes on a SupportTreeBuilder, under the same names, so planning code
// reads like the code that used to build directly. Elements it adds get
// recipe-local ids (LOCAL and up), which replay() maps to the builder's.
class BuildRecipe
{
    std::vector<BuildStep> m_steps;
    long m_pillars = 0, m_bridges = 0, m_diffbridges = 0, m_junctions = 0;

    void push(const BuildStep &s) { m_steps.emplace_back(s); }

public:
    // Far above any id a builder hands out, so a step can name a pillar that
    // exists already and one this recipe makes in the same field.
    static constexpr long LOCAL = 1L << 30;
    static bool is_local(long id) { return id >= LOCAL; }

    const std::vector<BuildStep> &steps() const { return m_steps; }
    bool empty() const { return m_steps.empty(); }

    long add_pillar(long head_id, double length);
    long add_pillar(const Vec3d &endpt, double height, double r_start, double r_end);
    void add_pillar_base(long pillar_id, double height, double radius);

    void add_junction(const Vec3d &pos, double r);
    size_t junctioncount() const { return size_t(LOCAL + m_junctions); }
    void own_junctions_from(size_t first, long pillar_id);

    Bridge add_bridge(const Vec3d &s, const Vec3d &e, double r, BridgeKind kind,
                      long pillar_id = SupportTreeNode::ID_UNSET);
    Bridge add_bridge_between(long pillar_id, const Vec3d &s, const Vec3d &e, double r,
                              BridgeKind kind, long from_pillar = SupportTreeNode::ID_UNSET);
    void add_bridge_to_pillar(long head_id, const Vec3d &endp, long pillar_id);
    DiffBridge add_diffbridge(const DiffBridge &br);

    void own_bridge(long bridge_id, long pillar_id);
    void own_diffbridge(long bridge_id, long pillar_id);

    void set_head_bridge(long head_id, long bridge_id);
    void set_head_diffbridge(long head_id, long bridge_id);
    void set_head_pillar(long head_id, long pillar_id);

    void add_anchor(double r_back, double r_pin, double width, double penetration,
                    const Vec3d &dir, const Vec3d &pos);
    void increment_bridges(long pillar_id);
};

// The builder ids replay() gave to what a recipe made, by recipe-local index.
struct RecipeIds
{
    std::vector<long> pillars, bridges, diffbridges, junctions;

    long pillar(long id) const
    {
        return BuildRecipe::is_local(id) ? pillars[size_t(id - BuildRecipe::LOCAL)] : id;
    }
};

// Makes the recipe's calls on the builder, in order.
RecipeIds replay(SupportTreeBuilder &builder, const BuildRecipe &recipe);

// A recipe that ends in a new pillar: `pillar` is its recipe-local id.
struct PillarRecipe
{
    BuildRecipe recipe;
    long        pillar = SupportTreeNode::ID_UNSET;
};

// How many pillars a head keeps as candidates to bridge to (spec "常數不開放
// 設定"). Not a setting. 8 at first; 4 since the timing of task 3.13 on
// China-dragon1 (design.md Open Questions 3): every candidate costs mesh tests.
inline constexpr size_t ROUTE_PILLAR_CANDIDATES = 4;

// One way to route a head, in full. Committed whole or not at all.
struct RoutePlan
{
    enum class Kind : uint8_t { to_pillar, own_pillar, ground_route, model_anchor };

    Kind        kind          = Kind::to_pillar;
    long        target_pillar = SupportTreeNode::ID_UNSET; // to_pillar: the pillar bridged to
    long        new_pillar    = SupportTreeNode::ID_UNSET; // recipe-local id of the pillar that
                                                           // goes into the pillar index, if any
    BuildRecipe recipe;
    size_t      ray_misses = 0;      // thin wall measurements with no exit, counted on commit
    bool        negative_anchor_width = false; // reported on commit
};

}} // namespace Slic3r::sla

#endif // SLA_ROUTEPLAN_HPP

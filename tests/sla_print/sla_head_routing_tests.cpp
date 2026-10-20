// Head routing in two phases (openspec change sla-head-bridge-clearance, task
// 3.11): routes are planned as builder calls and replayed on commit; steps 3(b)
// and 4 plan every head at once and commit them one by one in head id order.
//
// The engine cases run the default tree on a small model built here, a two
// tier table: supports under the top beside the ledge reach the ground (step
// 3), those over the ledge only reach the model (step 4), and the ledge's own
// underside reaches the ground again. Heads come in three sizes, mixed, so a
// thick head often has a thin pillar nearest to it.

#include <catch2/catch_test_macros.hpp>
#include <test_utils.hpp>

#include <memory>
#include <set>

#include <oneapi/tbb/global_control.h>

#include "libslic3r/Execution/ExecutionSeq.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/SLA/DefaultSupportTree.hpp"
#include "libslic3r/SLA/RoutePlan.hpp"
#include "libslic3r/SLA/SupportTree.hpp"
#include "libslic3r/SLA/SupportTreeBuilder.hpp"
#include "libslic3r/SLA/SupportTreeUtilsLegacy.hpp"

#include "sla_bracing_metrics.hpp"

using namespace Slic3r;
using namespace Slic3r::sla;

namespace {

// A 30 x 30 mm top at z 12 to 14, and under one side of it a 12 x 12 mm ledge
// at z 0 to 2, held up by a 2 x 2 mm post.
TriangleMesh make_two_tier()
{
    TriangleMesh mesh = make_cube(30., 30., 2.);
    mesh.translate(-15.f, -15.f, 12.f);

    TriangleMesh ledge = make_cube(12., 12., 2.);
    ledge.translate(-2.f, -6.f, 0.f);
    mesh.merge(ledge);

    TriangleMesh post = make_cube(2., 2., 10.);
    post.translate(3.f, -1.f, 2.f);
    mesh.merge(post);

    return mesh;
}

// Points on the undersides of the top (clear of the post) and of the ledge,
// their head back radii cycling through 0.25, 0.4 and 0.75 mm.
SupportPoints mixed_points()
{
    static const float radii[] = {0.25f, 0.4f, 0.75f};

    SupportPoints pts;
    auto add = [&pts](float x, float y, float z) {
        SupportPoint sp;
        sp.pos                 = Vec3f{x, y, z};
        sp.head_front_radius   = 0.2f;
        sp.head_back_radius_mm = radii[pts.size() % 3];
        pts.push_back(sp);
    };

    for (int i = 0; i < 14; ++i)
        for (int j = 0; j < 14; ++j) {
            const float x = -13.f + 2.f * float(i), y = -13.f + 2.f * float(j);
            if (x > 2.f && x < 6.f && y > -2.f && y < 2.f)
                continue; // where the post meets the top
            add(x, y, 12.f);
        }

    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j)
            add(-1.f + 2.5f * float(i), -5.f + 2.5f * float(j), 0.f);

    return pts;
}

struct Fixture
{
    TriangleMesh    mesh = make_two_tier();
    SupportableMesh sm{mesh.its, mixed_points(), SupportTreeConfig{}};
};

// The default tree, on one thread when `one_thread`, else on as many as there are.
std::unique_ptr<SupportTreeBuilder> build(const SupportableMesh &sm, bool one_thread)
{
    std::unique_ptr<tbb::global_control> limit;
    if (one_thread)
        limit = std::make_unique<tbb::global_control>(
            tbb::global_control::max_allowed_parallelism, 1);

    auto builder = std::make_unique<SupportTreeBuilder>();
    DefaultSupportTree::execute(*builder, sm);
    return builder;
}

bool same(const Vec3d &a, const Vec3d &b) { return (a - b).norm() < 1e-9; }

// The head a head bar comes from: the one whose junction it leaves, or the
// one whose short pillar it leaves from the top.
const Head *head_of_bar(const SupportTreeBuilder &b, const Bridge &bar)
{
    if (bar.owner_a >= 0) {
        const Pillar &own = b.pillar(bar.owner_a);
        return own.starts_from_head ? &b.head(unsigned(own.start_junction_id)) : nullptr;
    }
    for (const Head &h : b.heads())
        if (h.is_valid() && same(h.junction_point(), bar.startp))
            return &h;
    return nullptr;
}

// Make sure the fixture exercises what the cases below are about.
void require_exercised(const SupportTreeBuilder &b)
{
    std::set<double> sizes;
    for (const Head &h : b.heads())
        if (h.is_valid())
            sizes.insert(h.r_back_mm);
    REQUIRE(sizes.size() == 3);

    size_t head_bars = 0;
    for (const Bridge &br : b.bridges())
        head_bars += br.kind == BridgeKind::head;
    REQUIRE(head_bars > 0);
    REQUIRE(b.pillars().size() > 0);
}

} // namespace

TEST_CASE("a recipe replays onto the ids the builder hands out", "[head_routing]")
{
    // The builder holds elements already, so a recipe-local id passed through
    // unmapped would name the wrong one.
    SupportTreeBuilder b;
    b.add_head(0, Head{0.5, 0.2, 1., 0.5, DOWN, Vec3d{0., 0., 20.}});
    b.add_pillar(Vec3d{0., 0., 0.}, 5., 0.5);
    b.add_bridge(Vec3d{0., 0., 5.}, Vec3d{1., 0., 5.}, 0.5, BridgeKind::route);
    b.add_junction(Vec3d{0., 0., 5.}, 0.5);

    BuildRecipe r;
    const Bridge route = r.add_bridge(Vec3d{2., 0., 8.}, Vec3d{3., 0., 6.}, 0.4,
                                      BridgeKind::route);
    r.add_junction(Vec3d{3., 0., 6.}, 0.4);
    const long pid = r.add_pillar(Vec3d{3., 0., -2.}, 8., 0.4, 0.5);
    r.add_pillar_base(pid, 1., 2.);
    r.own_bridge(route.id, pid);
    r.set_head_bridge(0, route.id);

    // A short pillar under the head and a bar from its top to pillar 0, the
    // way a to_pillar plan lays it.
    const long own = r.add_pillar(0, 2.);
    const size_t first = r.junctioncount();
    r.add_junction(Vec3d{0., 0., 15.}, 0.5);
    r.own_junctions_from(first, own);
    r.add_bridge_between(0, Vec3d{0., 0., 15.}, Vec3d{0., 0., 5.}, 0.5, BridgeKind::head, own);
    r.increment_bridges(0);

    REQUIRE(BuildRecipe::is_local(pid));
    REQUIRE(BuildRecipe::is_local(route.id));

    const RecipeIds ids = replay(b, r);

    REQUIRE(ids.pillar(pid) == 1);
    REQUIRE(ids.pillar(own) == 2);
    REQUIRE(ids.pillar(0) == 0);

    REQUIRE(b.bridges().size() == 3);
    const Bridge &laid = b.bridges()[1];
    REQUIRE(laid.kind == BridgeKind::route);
    REQUIRE(laid.owner_b == 1);
    REQUIRE(b.head(0u).bridge_id == 1);

    const Bridge &bar = b.bridges()[2];
    REQUIRE(bar.kind == BridgeKind::head);
    REQUIRE(bar.owner_a == 2);
    REQUIRE(bar.owner_b == 0);

    REQUIRE(b.pillar(0).bridges == 1);
    REQUIRE(b.pillar(2).starts_from_head);
    REQUIRE(b.head(0u).pillar_id == 2);

    REQUIRE(b.pedestals().size() == 1);
    REQUIRE(b.pedestals()[0].pillar_id == 1);

    REQUIRE(b.junctions().size() == 3);
    REQUIRE(b.junctions()[0].pillar_id == SupportTreeNode::ID_UNSET);
    REQUIRE(b.junctions()[1].pillar_id == SupportTreeNode::ID_UNSET);
    REQUIRE(b.junctions()[2].pillar_id == 2);
}

TEST_CASE("a route with no way down makes nothing", "[head_routing]")
{
    // A junction above a slab that covers the whole plate: no route can be
    // planned, and the wrapper reports that and makes nothing.
    TriangleMesh mesh = make_cube(40., 40., 2.);
    mesh.translate(-20.f, -20.f, -8.f);
    SupportTreeConfig cfg;
    SupportableMesh   sm{mesh.its, {}, cfg};

    SupportTreeBuilder b;
    const auto [ok, pid] = connect_to_ground(ex_seq, b, sm,
                                             Junction{Vec3d{0., 0., 0.}, cfg.head_back_radius_mm},
                                             DOWN, cfg.head_back_radius_mm);
    REQUIRE_FALSE(ok);
    REQUIRE(pid == SupportTreeNode::ID_UNSET);
    REQUIRE(b.pillars().empty());
    REQUIRE(b.bridges().empty());
    REQUIRE(b.diffbridges().empty());
    REQUIRE(b.junctions().empty());
}

TEST_CASE("head routing does not depend on the thread count", "[head_routing]")
{
    Fixture f;

    const auto one = build(f.sm, true);
    require_exercised(*one);
    const BuilderDigests ref = builder_digests(*one);

    for (int run = 0; run < 3; ++run) {
        CAPTURE(run);
        const BuilderDigests d = builder_digests(*build(f.sm, false));
        REQUIRE(d.heads == ref.heads);
        REQUIRE(d.pillars == ref.pillars);
        REQUIRE(d.bridges == ref.bridges);
        REQUIRE(d.diffbridges == ref.diffbridges);
        REQUIRE(d.crossbridges == ref.crossbridges);
        REQUIRE(d.mesh == ref.mesh);
    }
}

TEST_CASE("a head leaves its junction one way at most", "[head_routing]")
{
    // Each route leaves the junction once: a bar from it (to a pillar, around
    // the model, or widening), or a pillar starting from the head (its own
    // pillar, a short one under a bar, or one down to an anchor). Two of them
    // are a bar left behind by a rejected candidate, or a bar and a pillar.
    // Prop bars are not counted: one may start on the top of a head's pillar,
    // which is its junction, later on in the auxiliary pillar stage.
    Fixture f;
    const auto    b = build(f.sm, false);
    require_exercised(*b);

    for (const Head &h : b->heads()) {
        if (!h.is_valid())
            continue;
        CAPTURE(h.id);

        const Vec3d jp   = h.junction_point();
        size_t      ways = 0;
        for (const Bridge &br : b->bridges())
            ways += (br.kind == BridgeKind::head || br.kind == BridgeKind::route) &&
                    same(br.startp, jp);
        for (const DiffBridge &br : b->diffbridges())
            ways += same(br.startp, jp);
        for (const Pillar &p : b->pillars())
            ways += p.starts_from_head && p.start_junction_id == h.id;

        REQUIRE(ways <= 1);
    }
}

TEST_CASE("no pillar takes more head bridges than the cap", "[head_routing]")
{
    // The cap is asked when a route is committed, on the counts the routes
    // before it left; a pillar's count is its head bars, one per route.
    Fixture f;
    const auto b = build(f.sm, false);
    require_exercised(*b);

    for (const Pillar &p : b->pillars()) {
        CAPTURE(p.id);

        unsigned bars = 0;
        for (const Bridge &br : b->bridges())
            bars += br.kind == BridgeKind::head && br.owner_b == p.id;

        REQUIRE(p.bridges == bars);
        REQUIRE(p.bridges <= f.sm.cfg.max_bridges_on_pillar);
    }
}

TEST_CASE("no head bridges to a pillar thinner than itself", "[head_routing]")
{
    Fixture f;
    const auto    b = build(f.sm, false);
    require_exercised(*b);

    for (const Bridge &br : b->bridges()) {
        if (br.kind != BridgeKind::head)
            continue;
        CAPTURE(br.id);

        REQUIRE(br.owner_b >= 0);
        const Head *h = head_of_bar(*b, br);
        REQUIRE(h != nullptr);
        REQUIRE(b->pillar(br.owner_b).r_start >= h->r_back_mm);
    }
}

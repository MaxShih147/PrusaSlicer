// Head routing in two phases (openspec change sla-head-bridge-clearance, task
// 3.11): routes are planned as builder calls and replayed on commit; steps 3(b)
// and 4 plan every head at once and commit them one by one in head id order.
// Task 4.8: which route is committed keeps the clearance where it can, and
// every contact it cannot keep apart is recorded.
//
// The engine cases run the default tree on a small model built here, a two
// tier table: supports under the top beside the ledge reach the ground (step
// 3), those over the ledge only reach the model (step 4), and the ledge's own
// underside reaches the ground again. Heads come in three sizes, mixed, so a
// thick head often has a thin pillar nearest to it. A dense version of it puts
// the supports closer than the thick heads can keep apart.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <test_utils.hpp>

#include <algorithm>
#include <iomanip>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>

#include <oneapi/tbb/global_control.h>

#include "libslic3r/Execution/ExecutionSeq.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/SLA/Clustering.hpp"
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

// The same sizes, 1.3 mm apart and one size per row along y: two thick heads
// side by side need 0.75 + 0.75 = 1.5 mm not to touch, so routes have to be
// kept close. Over the ledge and beside it, clear of the post, and under the
// ledge.
SupportPoints dense_points()
{
    static const float radii[] = {0.25f, 0.4f, 0.75f};

    SupportPoints pts;
    auto add = [&pts](float x, float y, float z, int row) {
        SupportPoint sp;
        sp.pos                 = Vec3f{x, y, z};
        sp.head_front_radius   = 0.2f;
        sp.head_back_radius_mm = radii[row % 3];
        pts.push_back(sp);
    };

    for (int i = 0; i < 11; ++i)
        for (int j = 0; j < 11; ++j) {
            const float x = -4.f + 1.3f * float(i), y = -6.5f + 1.3f * float(j);
            if (x > 1.5f && x < 6.5f && y > -2.5f && y < 2.5f)
                continue; // where the post meets the top
            add(x, y, 12.f, i);
        }

    for (int i = 0; i < 9; ++i)
        for (int j = 0; j < 9; ++j)
            add(-1.5f + 1.3f * float(i), -5.5f + 1.3f * float(j), 0.f, i);

    return pts;
}

struct Fixture
{
    TriangleMesh    mesh = make_two_tier();
    SupportableMesh sm{mesh.its, mixed_points(), SupportTreeConfig{}};
};

struct DenseFixture
{
    TriangleMesh    mesh = make_two_tier();
    SupportableMesh sm{mesh.its, dense_points(), SupportTreeConfig{}};
};

// The default tree, on one thread when `one_thread`, else on as many as there
// are; what the clearance rules left goes to `report` when given.
std::unique_ptr<SupportTreeBuilder> build(const SupportableMesh &sm, bool one_thread,
                                          ClearanceReport *report = nullptr)
{
    std::unique_ptr<tbb::global_control> limit;
    if (one_thread)
        limit = std::make_unique<tbb::global_control>(
            tbb::global_control::max_allowed_parallelism, 1);

    auto builder = std::make_unique<SupportTreeBuilder>();
    DefaultSupportTree::execute(*builder, sm, nullptr, report);
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

// ─── what the finished tree still has too close (task 4.8) ───────────────────
//
// The elements are read back from the builder and tied to the head whose path
// they are the way analyze_tree.py of the change's evidence does it, not
// taken from the engine's own index: a head bar by its head's own short pillar
// or by leaving the junction, a route by chaining from the junction, a pillar
// by its head, or by the route that comes down on it. Anchors are not read
// back, as the export leaves them out too.

using clearance::Prim;
using clearance::PrimKind;

struct Audit
{
    size_t contacts   = 0; // A, C and reverse C closer than the clearance
    size_t native     = 0; // ... of which a pillar against support point spacing
    size_t recorded   = 0; // ... of which an L1, L2 or centre pillar record names
    std::vector<std::string> unrecorded;
};

Audit audit(const SupportTreeBuilder &b, const ClearanceReport &report)
{
    using namespace clearance;

    std::map<long, const Head *> heads; // the valid ones, by id
    for (const Head &h : b.heads())
        if (h.is_valid())
            heads[h.id] = &h;

    // Head and route bars, with the head whose path each is.
    struct Bar { Vec3d s, e; double r; PrimKind kind; long a, b, path; };
    std::vector<Bar> bars;
    for (const Bridge &br : b.bridges())
        if (br.kind == BridgeKind::head || br.kind == BridgeKind::route)
            bars.push_back({br.startp, br.endp, br.r,
                            br.kind == BridgeKind::head ? PrimKind::head_bar : PrimKind::route_bar,
                            br.owner_a, br.owner_b, -1});
    for (const DiffBridge &br : b.diffbridges())
        bars.push_back({br.startp, br.endp, std::max(br.r, br.end_r), PrimKind::route_bar,
                        br.owner_a, br.owner_b, -1});

    for (Bar &bar : bars) {
        if (bar.kind == PrimKind::head_bar && bar.a >= 0 && b.pillar(bar.a).starts_from_head)
            bar.path = b.pillar(bar.a).start_junction_id;
        else
            for (const auto &[id, h] : heads)
                if (same(h->junction_point(), bar.s))
                    bar.path = id;
    }
    for (bool more = true; more;) { // routes chain on from where they reached
        more = false;
        for (Bar &bar : bars)
            if (bar.path < 0)
                for (const Bar &prev : bars)
                    if (prev.path >= 0 && same(prev.e, bar.s)) {
                        bar.path = prev.path;
                        more     = true;
                        break;
                    }
    }
    for (const Bar &bar : bars)
        REQUIRE(bar.path >= 0);

    auto pillar_path = [&](const Pillar &p) -> long {
        if (p.starts_from_head)
            return p.start_junction_id;
        for (const auto &[id, h] : heads)
            if (h->pillar_id == p.id)
                return id;
        for (const Bar &bar : bars) // a head bar reaching it is another head's
            if (bar.kind == PrimKind::route_bar && bar.b == p.id)
                return bar.path;
        return -1;
    };

    std::vector<Prim> prims;
    std::map<long, Prim> pinheads;
    for (const auto &[id, h] : heads) {
        long carrier = h->pillar_id;
        for (const Bar &bar : bars)
            if (carrier < 0 && bar.path == id && same(bar.s, h->junction_point()))
                carrier = bar.b;
        Prim p;
        p.cap      = pinhead_capsule(h->pos, h->junction_point(), h->r_back_mm);
        p.kind     = PrimKind::pinhead;
        p.head_id  = id;
        p.pillar_a = carrier;
        prims.push_back(p);
        pinheads[id] = p;
    }
    for (const Pillar &pl : b.pillars()) {
        if (pl.props_for >= 0)
            continue; // auxiliary pillars: the bracing's (task group 5)
        Prim p;
        p.cap      = pillar_capsule(pl.endpt, pl.height, pl.r_start, pl.r_end);
        p.kind     = PrimKind::pillar;
        p.head_id  = pillar_path(pl);
        p.pillar_a = pl.id;
        prims.push_back(p);
    }
    for (const Bar &bar : bars) {
        Prim p;
        p.cap      = bar_capsule(bar.s, bar.e, bar.r);
        p.kind     = bar.kind;
        p.head_id  = bar.path;
        p.pillar_a = bar.a;
        p.pillar_b = bar.b;
        prims.push_back(p);
    }

    // The pairs of paths the records name: a degraded route and what it
    // kept too close, a centre pillar's head and what it came too close to.
    std::set<std::pair<long, long>> named;
    auto name = [&named](long x, long y) { named.insert({std::min(x, y), std::max(x, y)}); };
    for (const RouteRecord &rec : report.routes)
        for (const Contact &ct : rec.contacts)
            name(rec.head, ct.with_head);
    for (const CentroidRecord &rec : report.centroid)
        name(pillar_path(b.pillar(rec.pillar)), rec.contact.with_head);

    auto axis_of = [&b](long id) {
        const Pillar &p = b.pillar(id);
        return JointAxis{Vec2d{p.endpt.x(), p.endpt.y()}, std::max(p.r_start, p.r_end)};
    };

    Audit out;
    for (size_t i = 0; i < prims.size(); ++i)
        for (size_t j = i + 1; j < prims.size(); ++j) {
            const Prim &x = prims[i], &y = prims[j];
            Contact     ct;
            if (!pair_contact(x, y, SAFETY_CLEARANCE, axis_of, ct))
                continue;
            ++out.contacts;

            // A pillar against a head natively that close to the pillar's
            // own head: support point spacing, out of scope.
            const Prim *pl = x.kind == PrimKind::pillar ? &x : y.kind == PrimKind::pillar ? &y : nullptr;
            const Prim *ph = x.kind == PrimKind::pinhead ? &x : y.kind == PrimKind::pinhead ? &y : nullptr;
            if (pl && ph && pinheads.count(pl->head_id)) {
                const Prim &grower = pinheads[pl->head_id];
                if (capsule_dist(grower.cap, ph->cap) <
                    grower.cap.r + ph->cap.r + SAFETY_CLEARANCE) {
                    ++out.native;
                    continue;
                }
            }

            if (named.count({std::min(x.head_id, y.head_id), std::max(x.head_id, y.head_id)})) {
                ++out.recorded;
                continue;
            }

            std::ostringstream os;
            os << std::fixed << std::setprecision(3) << kind_name(x.kind) << " of head "
               << x.head_id << " x " << kind_name(y.kind) << " of head " << y.head_id
               << ": dist " << ct.dist << " < " << ct.required << " at (" << ct.p_old.x()
               << "," << ct.p_old.y() << "," << ct.p_old.z() << ")";
            out.unrecorded.push_back(os.str());
        }

    return out;
}

// The records, as text, for comparing two runs.
std::string report_text(const ClearanceReport &rep)
{
    std::ostringstream os;
    os << std::setprecision(17) << rep.l1 << ' ' << rep.l2 << ' ' << rep.none << ' '
       << rep.native << '\n';
    auto contact = [&os](const clearance::Contact &ct) {
        os << ' ' << ct.p_new.transpose() << ' ' << ct.dist << ' ' << ct.required << ' '
           << int(ct.with) << ' ' << ct.with_head << ' ' << ct.with_pillar;
    };
    for (const RouteRecord &r : rep.routes) {
        os << r.head << ' ' << route_kind_name(r.kind) << ' ' << route_level_name(r.level);
        for (const clearance::Contact &ct : r.contacts)
            contact(ct);
        os << '\n';
    }
    for (const CentroidRecord &r : rep.centroid) {
        os << "centre " << r.pillar;
        contact(r.contact);
        os << '\n';
    }
    return os.str();
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
    // The tree and what the clearance rules recorded, which the dense
    // fixture has plenty of.
    auto same_every_run = [](const SupportableMesh &sm) {
        ClearanceReport one_report;
        const auto      one = build(sm, true, &one_report);
        require_exercised(*one);
        const BuilderDigests ref  = builder_digests(*one);
        const std::string    recs = report_text(one_report);

        for (int run = 0; run < 3; ++run) {
            CAPTURE(run);
            ClearanceReport      report;
            const BuilderDigests d = builder_digests(*build(sm, false, &report));
            REQUIRE(d.heads == ref.heads);
            REQUIRE(d.pillars == ref.pillars);
            REQUIRE(d.bridges == ref.bridges);
            REQUIRE(d.diffbridges == ref.diffbridges);
            REQUIRE(d.crossbridges == ref.crossbridges);
            REQUIRE(d.mesh == ref.mesh);
            REQUIRE(report_text(report) == recs);
        }
    };

    SECTION("mixed sizes") {
        Fixture f;
        same_every_run(f.sm);
    }
    SECTION("dense") {
        DenseFixture f;
        same_every_run(f.sm);
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

TEST_CASE("every contact left too close is recorded", "[head_routing]")
{
    // A, C and reverse C in the finished tree, judged at L0's clearance: each
    // one is a route kept at L1 or L2, or a centre pillar recorded, or a
    // pillar against support points natively that close.
    auto all_recorded = [](const SupportableMesh &sm) {
        ClearanceReport report;
        const auto      b = build(sm, false, &report);
        require_exercised(*b);

        const Audit a = audit(*b, report);
        CAPTURE(a.contacts, a.native, a.recorded, report.l1, report.l2, report.none,
                report.centroid.size());
        for (size_t i = 0; i < std::min<size_t>(a.unrecorded.size(), 10); ++i)
            UNSCOPED_INFO(a.unrecorded[i]);
        REQUIRE(a.unrecorded.empty());

        // Each record is what its level says.
        for (const RouteRecord &rec : report.routes) {
            CAPTURE(rec.head, route_level_name(rec.level));
            REQUIRE(rec.level != RouteLevel::L0);
            REQUIRE_FALSE(rec.contacts.empty());
            for (const clearance::Contact &ct : rec.contacts) {
                REQUIRE(ct.dist < ct.required);
                if (rec.level == RouteLevel::L1)
                    REQUIRE(ct.dist >= ct.required - clearance::SAFETY_CLEARANCE);
            }
        }
        size_t l1 = 0, l2 = 0;
        for (const RouteRecord &rec : report.routes)
            (rec.level == RouteLevel::L1 ? l1 : l2) += 1;
        REQUIRE(l1 == report.l1);
        REQUIRE(l2 == report.l2);

        return std::make_pair(a, report);
    };

    SECTION("mixed sizes") {
        Fixture f;
        all_recorded(f.sm);
    }

    SECTION("dense: routes are kept close, and say so") {
        DenseFixture f;
        const auto [a, report] = all_recorded(f.sm);
        REQUIRE(a.recorded > 0);
        REQUIRE(report.l1 + report.l2 > 0);
    }
}

namespace {

// A top whose middle is 3 mm higher than its sides, 2.2 mm wide: one head
// under the middle, two under the sides 1.15 mm out from it. Their heads point
// straight down, as given, so nothing searches for another angle.
struct SteppedFixture
{
    TriangleMesh mesh = [] {
        TriangleMesh left = make_cube(4.9, 6., 6.);
        left.translate(-6.f, -3.f, 17.f);
        TriangleMesh right = make_cube(4.9, 6., 6.);
        right.translate(1.1f, -3.f, 17.f);
        TriangleMesh middle = make_cube(2.2, 6., 3.);
        middle.translate(-1.1f, -3.f, 20.f);
        left.merge(right);
        left.merge(middle);
        return left;
    }();

    static SupportPoints points()
    {
        SupportPoints pts;
        for (const Vec3f &pos : {Vec3f{0.f, 0.f, 20.f}, Vec3f{1.15f, 0.f, 17.f},
                                 Vec3f{-1.15f, 0.f, 17.f}}) {
            SupportPoint sp;
            sp.pos                 = pos;
            sp.head_front_radius   = 0.2f;
            sp.head_back_radius_mm = 0.5f;
            sp.head_dir            = Vec3f{0.f, 0.f, -1.f};
            pts.push_back(sp);
        }
        return pts;
    }

    SupportableMesh sm{mesh.its, points(), SupportTreeConfig{}};
};

} // namespace

TEST_CASE("a centre pillar close to a head stays, recorded", "[head_routing]")
{
    // The middle head is the cluster's centre; its pillar runs down 1.15 mm
    // from the axes of the side heads, short of 0.5 + 0.5 + 0.2. The heads
    // themselves are 1.97 mm apart, so it is not support point spacing.
    SteppedFixture  f;
    ClearanceReport report;
    const auto      b = build(f.sm, false, &report);

    const Head &centre = b->head(0u);
    REQUIRE(centre.is_valid());
    REQUIRE(centre.pillar_id >= 0);

    // Where it would be with no clearance rule at all: straight down from
    // the head, grown from it.
    const Pillar &p = b->pillar(centre.pillar_id);
    REQUIRE(p.starts_from_head);
    REQUIRE(p.start_junction_id == 0);
    REQUIRE(p.endpt.x() == centre.junction_point().x());
    REQUIRE(p.endpt.y() == centre.junction_point().y());
    REQUIRE(p.startpoint().z() == Catch::Approx(centre.junction_point().z()));

    std::set<long> touched;
    for (const CentroidRecord &rec : report.centroid) {
        REQUIRE(rec.pillar == p.id);
        REQUIRE(rec.contact.with == clearance::PrimKind::pinhead);
        REQUIRE(rec.contact.dist == Catch::Approx(1.15).margin(1e-6));
        REQUIRE(rec.contact.required == Catch::Approx(1.2));
        touched.insert(rec.contact.with_head);
    }
    REQUIRE(touched == std::set<long>{1, 2});
    REQUIRE(report.native == 0);

    // Recorded, so nothing in the tree is left unrecorded.
    REQUIRE(audit(*b, report).unrecorded.empty());
}

TEST_CASE("the L2 routes of a tree go out as its warnings", "[head_routing][sla_io]")
{
    // What create_support_tree() hands the export (task 6.1) is the L2 records
    // of the same run, head by head, with each head by its exported position.
    auto check = [](const SupportableMesh &sm) {
        ClearanceReport report;
        const auto      b = build(sm, false, &report);
        require_exercised(*b);

        JobController       ctl;
        SupportTreeElements els;
        create_support_tree(sm, ctl, nullptr, nullptr, nullptr, &els);

        std::vector<const RouteRecord *> l2;
        for (const RouteRecord &rec : report.routes)
            if (rec.level == RouteLevel::L2)
                l2.push_back(&rec);
        CAPTURE(report.l1, report.l2);
        REQUIRE(l2.size() == report.l2);
        REQUIRE(els.warnings.size() == l2.size());

        for (size_t i = 0; i < l2.size(); ++i) {
            const RouteRecord &rec = *l2[i];
            const TreeWarning &w   = els.warnings[i];
            CAPTURE(i, rec.head, w.head);
            REQUIRE(w.code == FALLBACK_INTERSECTION_RETAINED);
            REQUIRE(w.route == route_kind_name(rec.kind));
            REQUIRE(w.head >= 0);
            REQUIRE(size_t(w.head) < els.heads.size());
            REQUIRE(same(els.heads[size_t(w.head)].pos, b->head(unsigned(rec.head)).pos));

            REQUIRE(w.contacts.size() == rec.contacts.size());
            bool touches = false; // L1 failed, so something is closer than the radii
            for (size_t k = 0; k < w.contacts.size(); ++k) {
                const TreeWarningContact &c  = w.contacts[k];
                const clearance::Contact &ct = rec.contacts[k];
                REQUIRE(same(c.pos, ct.p_new));
                REQUIRE(c.with == clearance::kind_name(ct.with));
                REQUIRE(c.dist == ct.dist);
                REQUIRE(c.required == ct.radii);
                REQUIRE(c.required == Catch::Approx(ct.required - clearance::SAFETY_CLEARANCE));
                touches = touches || c.dist < c.required;
            }
            REQUIRE(touches);
        }
        return els.warnings.size();
    };

    size_t warned = 0;
    SECTION("mixed sizes") {
        Fixture f;
        warned += check(f.sm);
    }
    SECTION("dense") {
        DenseFixture f;
        warned += check(f.sm);
    }
    // Neither fixture is promised an L2; say when the comparison was empty.
    if (warned == 0)
        WARN("no L2 route in this fixture: only the absence of warnings was checked");
}

namespace {

// A roof at z 10 to 12 over x -12 to 4, held up by a wall at x 0 to 4 that
// stands on the ground. Zero elevation: a pillar's foot is on the ground
// beside the wall.
//
// Five heads under the roof, pointing straight down, so they are placed as
// given:
//   0  X at (-5, 0)        lone, grows its own pillar
//   1  Y at (-5.2, 4.2)    lone, grows its own pillar
//   2  h1 at (-1.2, -3)    } one cluster, centred on 3. The heads clear the
//   3  cc at (-0.45, 0)    } wall by more than r + safety distance, so they
//   4  h2 at (-1.2, 3)     } reach the ground. cc, 0.2 mm thin, is 0.45 mm
//                            from the wall: closer than the pillar base safety
//                            distance (0.5 mm), and its pillar searches only
//                            towards the wall (+x, the azimuth of straight
//                            down), so it cannot be made.
// X is the pillar nearest the centre head; Y is nearer to h2 than X is.
struct FailedCentreFixture
{
    TriangleMesh mesh = [] {
        TriangleMesh wall = make_cube(4., 18., 12.);
        wall.translate(0.f, -8.f, 0.f);
        TriangleMesh roof = make_cube(16., 18., 2.);
        roof.translate(-12.f, -8.f, 10.f);
        wall.merge(roof);
        return wall;
    }();

    static SupportPoints points()
    {
        SupportPoints pts;
        auto add = [&pts](float x, float y, float r_back) {
            SupportPoint sp;
            sp.pos                 = Vec3f{x, y, 10.f};
            sp.head_front_radius   = std::min(0.2f, r_back / 2.f);
            sp.head_back_radius_mm = r_back;
            sp.head_dir            = Vec3f{0.f, 0.f, -1.f};
            pts.push_back(sp);
        };
        add(-5.f, 0.f, 0.5f);   // X
        add(-5.2f, 4.2f, 0.5f); // Y
        add(-1.2f, -3.f, 0.5f); // h1
        add(-0.45f, 0.f, 0.2f); // cc
        add(-1.2f, 3.f, 0.5f);  // h2
        return pts;
    }

    static SupportTreeConfig config()
    {
        SupportTreeConfig cfg;
        cfg.object_elevation_mm = 0.;
        return cfg;
    }

    SupportableMesh sm{mesh.its, points(), config()};

    FailedCentreFixture()
    {
        // The foot of a pillar with no base goes no lower than the ground,
        // so it stays beside the wall.
        sm.pad_cfg.wall_thickness_mm = 0.;
    }
};

} // namespace

TEST_CASE("a cluster whose centre pillar failed offers no centre pillar",
          "[head_routing]")
{
    // Spec "中心柱建立失敗": the other heads of the cluster MUST NOT take any
    // pillar as the centre. The old lookup took the pillar nearest the centre
    // head (X), and put it first; without a centre, h2 goes by distance to Y.
    FailedCentreFixture f;
    const auto          b = build(f.sm, false);

    // cc is routed in step 4 instead, and may be lost there: that is not
    // this rule's, so only the others have to be there.
    const unsigned X = 0, Y = 1, H1 = 2, CC = 3, H2 = 4;
    for (unsigned id : {X, Y, H1, H2}) {
        CAPTURE(id);
        REQUIRE(b->head(id).is_valid());
    }

    auto jp2d = [&b](unsigned id) {
        const Vec3d p = b->head(id).junction_point();
        return Vec2d{p.x(), p.y()};
    };

    // The fixture is what it says. h1, cc and h2 make one cluster, as
    // classify() makes them (bases 2 + 2 mm apart in plan), centred on cc;
    // X and Y are clusters of their own.
    const std::vector<unsigned> ground = {X, Y, H1, CC, H2};
    const ClusteredPoints clusters = cluster(
        ground, [&b](unsigned id) { return b->head(id).junction_point(); },
        [&f](const PointIndexEl &e1, const PointIndexEl &e2) {
            const double bases = resolved_base_radius_mm(f.sm, long(e1.second)) +
                                 resolved_base_radius_mm(f.sm, long(e2.second));
            return (to_2d(e1.first) - to_2d(e2.first)).norm() < bases &&
                   (e1.first - e2.first).norm() < f.sm.cfg.max_bridge_length_mm;
        },
        f.sm.cfg.max_bridges_on_pillar);
    REQUIRE(clusters.size() == 3);
    const ClusterEl *mine = nullptr;
    for (const ClusterEl &cl : clusters)
        if (std::find(cl.begin(), cl.end(), CC) != cl.end())
            mine = &cl;
    REQUIRE(mine != nullptr);
    REQUIRE(std::set<unsigned>(mine->begin(), mine->end()) == std::set<unsigned>{H1, CC, H2});
    const long centre = cluster_centroid(
        *mine, [&b](size_t idx) { return b->head(unsigned(idx)).junction_point(); },
        [](const Vec3d &p, const Vec3d &q) { return (to_2d(p) - to_2d(q)).norm(); });
    REQUIRE((*mine)[size_t(centre)] == CC);

    // ... and the centre pillar cannot be made: the call create_center_pillar() makes.
    const Head &cc = b->head(CC);
    REQUIRE_FALSE(plan_ground_pillar(ex_seq, f.sm, cc.junction_point(), cc.dir, cc.r_back_mm,
                                     cc.r_back_mm, long(CC))
                      .has_value());

    // X and Y stand on their own pillars.
    auto grown_by = [&b](unsigned id) {
        for (const Pillar &p : b->pillars())
            if (p.starts_from_head && p.start_junction_id == long(id))
                return p.id;
        return long(SupportTreeNode::ID_UNSET);
    };
    const long pX = grown_by(X), pY = grown_by(Y);
    REQUIRE(pX >= 0);
    REQUIRE(pY >= 0);

    // What tells the two rules apart: X is the pillar nearest the centre
    // head, Y the one nearest h2.
    REQUIRE((jp2d(CC) - jp2d(X)).norm() < (jp2d(CC) - jp2d(Y)).norm());
    REQUIRE((jp2d(H2) - jp2d(Y)).norm() < (jp2d(H2) - jp2d(X)).norm());

    // h2 goes to Y: nothing stood in for the centre pillar ahead of it.
    std::vector<long> h2_bars;
    for (const Bridge &br : b->bridges())
        if (br.kind == BridgeKind::head) {
            const Head *h = head_of_bar(*b, br);
            if (h && h->id == long(H2))
                h2_bars.push_back(br.owner_b);
        }
    REQUIRE(h2_bars == std::vector<long>{pY});
}

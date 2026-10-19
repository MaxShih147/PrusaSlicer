// Exchange formats (openspec change sla-bracing-planar-topology, task group 5).
//
// support-tree (version 2) gains bridges[].kind, always written, and
// pillars[].pairs; prior-pillar (version 1) gains pillars[].pairs and
// attached_to[].pairs, and still reads a list that has no pairs. The reference
// models are checked by the hidden "[.bracing_io]" case next to the bracing
// baseline, which needs the same plan file.

#include <catch2/catch_test_macros.hpp>

#include <map>
#include <set>
#include <string>

#include <nlohmann/json.hpp>

#include "libslic3r/SLA/PriorPillarIO.hpp"
#include "libslic3r/SLA/SupportTreeIO.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"

#include "sla_test_utils.hpp"

using namespace Slic3r;
using namespace Slic3r::sla;

namespace {

PriorPillars parse_prior(const std::string &text)
{
    PriorPillars out;
    std::string  err;
    REQUIRE(prior_pillars_from_string(text, out, err));
    REQUIRE(err.empty());
    return out;
}

// Every bar has a kind, and a known one; the count of each.
std::map<std::string, size_t> kinds_of(const nlohmann::json &tree)
{
    static const std::set<std::string> known = {"truss", "head", "prop", "route"};
    std::map<std::string, size_t> n;
    for (const nlohmann::json &b : tree.at("bridges")) {
        REQUIRE(b.contains("kind"));
        REQUIRE(b.at("kind").is_string());
        const std::string k = b.at("kind").get<std::string>();
        REQUIRE(known.count(k) == 1);
        ++n[k];
    }
    return n;
}

} // namespace

TEST_CASE("an old prior pillar list takes its pairs from links", "[sla_io]")
{
    const PriorPillars ps = parse_prior(R"({"version":1,"pillars":[
        {"id":1,"endpoint":[0,0,0],"height":20,"r_start":0.5,"links":2,"bridges":0},
        {"id":2,"endpoint":[5,0,0],"height":20,"r_start":0.5},
        {"id":3,"endpoint":[9,0,0],"height":20,"r_start":0.5,"links":2,"pairs":1},
        {"id":4,"endpoint":[0,5,0],"height":20,"r_start":0.5,"links":1,"pairs":3}]})");
    REQUIRE(ps.size() == 4);
    REQUIRE(ps[0].links == 2);
    REQUIRE(ps[0].pairs == 2); // absent: links
    REQUIRE(ps[1].links == 0);
    REQUIRE(ps[1].pairs == 0); // absent, and so are links
    REQUIRE(ps[2].pairs == 2); // fewer than links: lifted to links
    REQUIRE(ps[3].pairs == 3); // as given
}

TEST_CASE("a prior pillar list with a bad pairs takes links", "[sla_io]")
{
    // null is what JSON.stringify makes of NaN. None of these may throw, wrap
    // round, or fail the list: each pillar falls back to its links.
    const PriorPillars ps = parse_prior(R"({"version":1,"pillars":[
        {"id":1,"endpoint":[0,0,0],"height":20,"r_start":0.5,"links":2,"pairs":null},
        {"id":2,"endpoint":[1,0,0],"height":20,"r_start":0.5,"links":2,"pairs":-1},
        {"id":3,"endpoint":[2,0,0],"height":20,"r_start":0.5,"links":2,"pairs":2.5},
        {"id":4,"endpoint":[3,0,0],"height":20,"r_start":0.5,"links":2,"pairs":"3"},
        {"id":5,"endpoint":[4,0,0],"height":20,"r_start":0.5,"links":2,"pairs":true},
        {"id":6,"endpoint":[5,0,0],"height":20,"r_start":0.5,"links":2,"pairs":4294967296},
        {"id":7,"endpoint":[6,0,0],"height":20,"r_start":0.5,"links":2,"pairs":[3]},
        {"id":8,"endpoint":[7,0,0],"height":20,"r_start":0.5,"links":2,"pairs":4294967295}]})");
    REQUIRE(ps.size() == 8);
    for (size_t i = 0; i + 1 < ps.size(); ++i) {
        CAPTURE(ps[i].id);
        REQUIRE(ps[i].pairs == 2);
    }
    REQUIRE(ps[7].pairs == 4294967295u); // the largest it can hold is taken as given
}

TEST_CASE("the prior pillar export writes pairs and stays version 1", "[sla_io]")
{
    PriorPillar p;
    p.id = 7; p.endpoint = {1., 2., 0.}; p.height = 30.;
    p.r_start = p.r_end = 0.5;
    p.links = 1; p.pairs = 3;
    const PriorAttachments att = {PriorAttachment{500, 2, 0, 4}};

    const nlohmann::json doc = nlohmann::json::parse(prior_pillars_to_string({p}, 5., att));
    REQUIRE(doc.at("version") == 1);
    REQUIRE(doc.at("pillars").at(0).at("pairs") == 3);
    REQUIRE(doc.at("attached_to").at(0).at("prior_id") == 500);
    REQUIRE(doc.at("attached_to").at(0).at("links") == 2);
    REQUIRE(doc.at("attached_to").at(0).at("pairs") == 4);

    // What one generation writes, the next reads back unchanged.
    const PriorPillars back = parse_prior(doc.dump());
    REQUIRE(back.size() == 1);
    REQUIRE(back[0].links == 1);
    REQUIRE(back[0].pairs == 3);
}

TEST_CASE("the support tree export names every kind and stays version 2", "[sla_io]")
{
    SupportTreeElements els;
    TreePillar tp;
    tp.height = 20.; tp.r_start = tp.r_end = 0.5;
    tp.links = 1; tp.pairs = 2;
    els.pillars.push_back(tp);

    const BridgeKind order[] = {BridgeKind::truss, BridgeKind::head, BridgeKind::prop,
                                BridgeKind::route};
    for (BridgeKind k : order) {
        TreeBridge b;
        b.r = b.end_r = 0.5;
        b.kind = k;
        els.bridges.push_back(b);
    }

    const nlohmann::json doc = nlohmann::json::parse(support_tree_to_string(els, 5.));
    REQUIRE(doc.at("version") == 2);
    REQUIRE(doc.at("pillars").at(0).at("links") == 1);
    REQUIRE(doc.at("pillars").at(0).at("pairs") == 2);
    const auto &bs = doc.at("bridges");
    REQUIRE(bs.size() == 4);
    REQUIRE(bs.at(0).at("kind") == "truss");
    REQUIRE(bs.at(1).at("kind") == "head");
    REQUIRE(bs.at(2).at("kind") == "prop");
    REQUIRE(bs.at(3).at("kind") == "route");
}

TEST_CASE("a generation onto prior pillars exports kinds and pairs", "[sla_io]")
{
    // A cube lifted 10 mm, with a row of earlier pillars along one side, so
    // the run both grows a tree of its own and braces to the old row.
    SupportTreeConfig cfg;
    cfg.object_elevation_mm = 10.;
    TriangleMesh mesh = load_model("20mm_cube.obj");
    const auto bb = mesh.bounding_box();
    const double gnd = bb.min.z() - cfg.object_elevation_mm;

    auto slicegrid = grid(float(gnd), float(bb.max.z()), 0.05f);
    auto slices    = slice_mesh_ex(mesh.its, slicegrid, CLOSING_RADIUS);
    SupportableMesh sm{mesh.its, {}, cfg};
    SupportPointGeneratorConfig autogencfg;
    auto gen = prepare_generator_data(std::move(slices), slicegrid);
    sm.pts = move_on_mesh_surface(generate_support_points(gen, autogencfg), sm.emesh,
                                  (slicegrid[1] - slicegrid[0]) +
                                      std::numeric_limits<float>::epsilon());
    sm.zoffset = bb.min.z();
    REQUIRE_FALSE(sm.pts.empty());

    // The old list as the caller would hand it in: written before pairs existed.
    std::string old_list = R"({"version":1,"pillars":[)";
    for (int i = 0; i < 4; ++i) {
        if (i) old_list += ",";
        old_list += "{\"id\":" + std::to_string(500 + i) + ",\"endpoint\":[" +
                    std::to_string(bb.min.x() - 3.) + "," +
                    std::to_string(bb.min.y() + 2. + 5. * i) + "," + std::to_string(gnd) +
                    "],\"height\":25,\"r_start\":0.5,\"links\":" + (i == 0 ? "1" : "0") + "}";
    }
    old_list += "]}";
    sm.prior = parse_prior(old_list);
    REQUIRE(sm.prior[0].pairs == 1);

    JobController       ctl;
    PriorPillars        grown;
    PriorAttachments    attached;
    SupportTreeElements els;
    create_support_tree(sm, ctl, &grown, &attached, nullptr, &els);
    REQUIRE_FALSE(els.bridges.empty());
    REQUIRE_FALSE(attached.empty());

    const nlohmann::json tree = nlohmann::json::parse(support_tree_to_string(els, 10.));
    REQUIRE(tree.at("version") == 2);
    const auto kinds = kinds_of(tree);
    REQUIRE(kinds.count("truss") == 1);
    REQUIRE(tree.at("pillars").size() == els.pillars.size());
    for (const nlohmann::json &p : tree.at("pillars")) {
        REQUIRE(p.contains("pairs"));
        REQUIRE(p.at("pairs").get<unsigned>() >= p.at("links").get<unsigned>());
    }

    const nlohmann::json list =
        nlohmann::json::parse(prior_pillars_to_string(grown, 10., attached));
    REQUIRE(list.at("version") == 1);
    REQUIRE(list.at("pillars").size() == grown.size());
    for (size_t i = 0; i < grown.size(); ++i)
        REQUIRE(list.at("pillars").at(i).at("pairs") == grown[i].pairs);
    REQUIRE(list.at("attached_to").size() == attached.size());
    for (size_t i = 0; i < attached.size(); ++i) {
        const nlohmann::json &a = list.at("attached_to").at(i);
        REQUIRE(a.at("pairs") == attached[i].pairs);
        // Each frozen pillar braced to has at least one more pair than it came with.
        const unsigned before = attached[i].prior_id == 500 ? 1u : 0u;
        REQUIRE(attached[i].pairs > before);
        REQUIRE(attached[i].pairs >= attached[i].links);
    }
}

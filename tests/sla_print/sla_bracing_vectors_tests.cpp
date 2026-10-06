// Shared bracing test vectors (openspec change sla-bracing-planar-topology,
// task group 6).
//
// tests/data/sla_bracing_vectors/ holds the canonical cases of the bracing
// rules, one JSON file each; the DS-online port runs the very same files. Each
// is run through the BracingPlanner - greedy, rescue, safety net, no props -
// with the model never in the way, and its accepted groups (in build order,
// with the bars each got) and refusals (in the order tried, with the reason)
// must match "expect" exactly.
//
// MANIFEST.sha256 lists every vector's SHA-256 ("<sha256>  <file>", LF). The
// files are LF too: a CRLF checkout changes every hash, and is refused with a
// message saying so. After changing a vector, regenerate the manifest from Git
// Bash, then check it:
//
//   cd tests/data/sla_bracing_vectors && sha256sum -t *.json > MANIFEST.sha256
//   cd tests/data/sla_bracing_vectors && sha256sum -c MANIFEST.sha256
//
// BRACING_VECTORS_DUMP=1 prints what the planner decides for each vector, as
// "expect" would hold it.
//
// "[bracing_engine]" is the engine-level check of the auxiliary pillar search
// (task 6.5), and of the owner a route's bars get (code review item 4).

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "libslic3r/Execution/ExecutionSeq.hpp"
#include "libslic3r/SLA/BracingPlanner.hpp"
#include "libslic3r/SLA/SupportTree.hpp"
#include "libslic3r/SLA/SupportTreeUtilsLegacy.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include "test_utils.hpp"

using namespace Slic3r;
using namespace Slic3r::sla;
using namespace Slic3r::sla::bracing;
namespace fs = std::filesystem;

namespace {

// ─── SHA-256 (FIPS 180-4) ────────────────────────────────────────────────────

std::string sha256_hex(const std::string &data)
{
    static const uint32_t K[64] = {
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
        0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
        0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
        0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
    uint32_t h[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                     0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};

    std::string msg = data;
    const uint64_t bits = uint64_t(data.size()) * 8;
    msg.push_back(char(0x80));
    while (msg.size() % 64 != 56) msg.push_back(char(0));
    for (int i = 7; i >= 0; --i) msg.push_back(char((bits >> (8 * i)) & 0xff));

    auto rotr = [](uint32_t x, int n) { return (x >> n) | (x << (32 - n)); };
    for (size_t off = 0; off < msg.size(); off += 64) {
        uint32_t w[64];
        for (int i = 0; i < 16; ++i)
            w[i] = uint32_t(uint8_t(msg[off + 4 * i])) << 24 |
                   uint32_t(uint8_t(msg[off + 4 * i + 1])) << 16 |
                   uint32_t(uint8_t(msg[off + 4 * i + 2])) << 8 |
                   uint32_t(uint8_t(msg[off + 4 * i + 3]));
        for (int i = 16; i < 64; ++i) {
            const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
            const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }
        uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
        for (int i = 0; i < 64; ++i) {
            const uint32_t t1 = hh + (rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25)) +
                                ((e & f) ^ (~e & g)) + K[i] + w[i];
            const uint32_t t2 = (rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22)) +
                                ((a & b) ^ (a & c) ^ (b & c));
            hh = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
        }
        h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
    }

    static const char *hex = "0123456789abcdef";
    std::string out;
    for (uint32_t v : h)
        for (int i = 28; i >= 0; i -= 4) out.push_back(hex[(v >> i) & 0xf]);
    return out;
}

// ─── the vector directory ────────────────────────────────────────────────────

fs::path vector_dir() { return fs::path(TEST_DATA_DIR) / "sla_bracing_vectors"; }

std::string read_bytes(const fs::path &p)
{
    std::ifstream in(p, std::ios::binary);
    REQUIRE(in.good());
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// Every vector file in the directory, by name, as bytes.
std::map<std::string, std::string> vector_files()
{
    std::map<std::string, std::string> out;
    for (const fs::directory_entry &e : fs::directory_iterator(vector_dir()))
        if (e.is_regular_file() && e.path().extension() == ".json")
            out[e.path().filename().string()] = read_bytes(e.path());
    return out;
}

// What is wrong between a manifest and the files it should describe; empty
// when they agree.
std::vector<std::string> manifest_problems(const std::string &manifest,
                                           const std::map<std::string, std::string> &files)
{
    std::vector<std::string> out;
    if (manifest.find('\r') != std::string::npos)
        out.push_back("MANIFEST.sha256 has CR line ends (a CRLF checkout?)");

    std::set<std::string> listed;
    std::istringstream lines(manifest);
    std::string line;
    while (std::getline(lines, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        // "<64 hex>  <name>", as sha256sum writes it.
        if (line.size() < 67 || line.compare(64, 2, "  ") != 0) {
            out.push_back("malformed line: " + line);
            continue;
        }
        const std::string hash = line.substr(0, 64), name = line.substr(66);
        listed.insert(name);
        const auto it = files.find(name);
        if (it == files.end()) {
            out.push_back("listed but missing: " + name);
        } else if (sha256_hex(it->second) != hash) {
            out.push_back("hash differs: " + name);
        }
    }
    for (const auto &[name, bytes] : files) {
        if (listed.count(name) == 0) out.push_back("not listed: " + name);
        if (bytes.find('\r') != std::string::npos)
            out.push_back(name + " has CR line ends (a CRLF checkout?)");
    }
    return out;
}

// ─── running a vector ────────────────────────────────────────────────────────

Vec3d vec3(const nlohmann::json &j) { return {j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>()}; }

PillarConnectionMode mode_of(const std::string &s)
{
    if (s == "zigzag") return PillarConnectionMode::zigzag;
    if (s == "cross") return PillarConnectionMode::cross;
    REQUIRE(s == "dynamic");
    return PillarConnectionMode::dynamic;
}

struct Variant
{
    double pillar_dz = 0.;  // added to every pillar's foot
    double ground_dz = 0.;  // added to ground_z
    double slope_drad = 0.; // added to the bridge slope
};

// The decisions as "expect" holds them.
nlohmann::json run_vector(const nlohmann::json &v, const Variant &var = {})
{
    const nlohmann::json &c = v.at("cfg");

    // The constants of the rules are the engine's; a vector only restates them.
    REQUIRE(c.at("safety_clearance").get<double>() == SAFETY_CLEARANCE_MM);
    REQUIRE(c.at("min_shared_angle_deg").get<double>() == MIN_SHARED_ANGLE_DEG);
    REQUIRE(c.at("pair_cap").get<unsigned>() == PAIR_CAP);
    REQUIRE(c.at("rescue_height").get<double>() == RESCUE_HEIGHT_MM);

    PlannerConfig pc;
    pc.max_link_distance      = c.at("max_link_distance").get<double>();
    pc.min_pillar_distance    = c.at("min_pillar_distance").get<double>();
    pc.truss.bridge_slope_rad = c.at("bridge_slope_deg").get<double>() * PI / 180. + var.slope_drad;
    pc.truss.ground_z         = c.at("ground_z").get<double>() + var.ground_dz;
    pc.truss.base_height      = c.at("base_height").get<double>();
    pc.truss.base_radius      = c.at("base_radius").get<double>();
    pc.truss.mode             = mode_of(c.at("connection_mode").get<std::string>());

    BracingPlanner planner(pc, [](const Vec3d &, const Vec3d &, double) { return true; });

    for (const nlohmann::json &pj : v.at("pillars")) {
        PillarGeom p;
        p.id     = pj.at("id").get<long>();
        const Vec3d e = vec3(pj.at("endpoint"));
        p.xy     = {e.x(), e.y()};
        p.foot_z = e.z() + var.pillar_dz;
        p.height = pj.at("height").get<double>();
        p.r      = pj.at("r").get<double>();
        p.links  = pj.at("links").get<unsigned>();
        p.pairs  = pj.at("pairs").get<unsigned>();
        p.frozen = pj.at("frozen").get<bool>();
        planner.add_pillar(p);
    }
    for (const nlohmann::json &bj : v.at("bars")) {
        const Segment3 seg{vec3(bj.at("startp")), vec3(bj.at("endp")), bj.at("r").get<double>()};
        const long a = bj.value("a", -1L), b = bj.value("b", -1L);
        if (bj.value("kind", std::string{}) == "truss")
            planner.add_truss_bar(a, b, seg);
        else
            planner.add_obstacle_bar({seg, a, b});
    }

    nlohmann::json accepted = nlohmann::json::array(), rejected = nlohmann::json::array();
    auto record = [&](const std::vector<Decision> &ds, const char *phase) {
        for (const Decision &d : ds) {
            // No candidate joins two frozen pillars, whatever the phase.
            REQUIRE_FALSE((planner.pillar(d.edge.a).frozen && planner.pillar(d.edge.b).frozen));
            const nlohmann::json pair = {d.edge.a, d.edge.b};
            if (d.reason == Reason::accepted) {
                accepted.push_back({{"phase", phase}, {"pair", pair}, {"bars_built", d.bars_built}});
            } else {
                // The model is never in the way, and every vector candidate is
                // in range: the vector reasons only.
                REQUIRE(d.reason != Reason::mesh_hit);
                REQUIRE(d.reason != Reason::out_of_range);
                rejected.push_back({{"phase", phase}, {"pair", pair}, {"reason", reason_name(d.reason)}});
            }
        }
    };
    record(planner.run_greedy(), "greedy");
    record(planner.run_rescue(), "rescue");
    record(planner.run_fallback(), "fallback");

    nlohmann::json after = nlohmann::json::array();
    for (const nlohmann::json &pj : v.at("pillars")) {
        const PillarGeom &p = planner.pillar(pj.at("id").get<long>());
        after.push_back({{"id", p.id}, {"links", p.links}, {"pairs", p.pairs}});
    }
    return {{"accepted", accepted}, {"rejected", rejected}, {"pillars_after", after}};
}

nlohmann::json load_vector(const std::string &bytes) { return nlohmann::json::parse(bytes); }

} // namespace

TEST_CASE("bracing vectors: the manifest matches the files", "[bracing_vectors]")
{
    const auto files = vector_files();
    REQUIRE(files.size() >= 30);
    const std::vector<std::string> problems =
        manifest_problems(read_bytes(vector_dir() / "MANIFEST.sha256"), files);
    for (const std::string &p : problems) UNSCOPED_INFO(p);
    REQUIRE(problems.empty());
}

TEST_CASE("bracing vectors: a changed vector no longer matches the manifest", "[bracing_vectors]")
{
    REQUIRE(sha256_hex("") == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    REQUIRE(sha256_hex("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");

    auto files = vector_files();
    const std::string manifest = read_bytes(vector_dir() / "MANIFEST.sha256");
    REQUIRE(manifest_problems(manifest, files).empty());

    // One byte changed, the manifest left as it was.
    std::string &bytes = files.begin()->second;
    bytes[bytes.find("\"height\":") + 9] ^= 1;
    REQUIRE(manifest_problems(manifest, files).size() == 1);

    // The same file checked out with CRLF.
    files = vector_files();
    std::string crlf;
    for (char ch : files.begin()->second) {
        if (ch == '\n') crlf.push_back('\r');
        crlf.push_back(ch);
    }
    files.begin()->second = crlf;
    REQUIRE(manifest_problems(manifest, files).size() == 2);
}

TEST_CASE("bracing vectors: every vector decides as expected", "[bracing_vectors]")
{
    const bool dump = std::getenv("BRACING_VECTORS_DUMP") != nullptr;

    for (const auto &[name, bytes] : vector_files()) {
        CAPTURE(name);
        const nlohmann::json v = load_vector(bytes);
        REQUIRE(v.at("id").get<std::string>() + ".json" == name);

        const nlohmann::json got = run_vector(v);
        if (dump) {
            std::cout << nlohmann::json{{"id", v.at("id")}, {"expect", got}}.dump() << std::endl;
            continue;
        }

        const nlohmann::json &exp = v.at("expect");
        CAPTURE(got.dump());
        REQUIRE(got.at("accepted") == exp.at("accepted"));
        REQUIRE(got.at("rejected") == exp.at("rejected"));
        if (exp.contains("pillars_after"))
            for (const nlohmann::json &want : exp.at("pillars_after")) {
                bool found = false;
                for (const nlohmann::json &p : got.at("pillars_after"))
                    if (p.at("id") == want.at("id")) {
                        REQUIRE(p == want);
                        found = true;
                    }
                REQUIRE(found);
            }

        // No decision hangs on the last digit of tan(): a port whose tan()
        // differs in the last bits decides the same.
        for (double d : {-1e-9, 1e-9}) {
            const nlohmann::json nudged = run_vector(v, Variant{0., 0., d});
            REQUIRE(nudged.at("accepted") == got.at("accepted"));
            REQUIRE(nudged.at("rejected") == got.at("rejected"));
        }

        // Mixing frames - pillars left in world Z, or ground_z left in world Z,
        // while the rest is in engine Z - must show in the decisions.
        if (v.contains("frame_check")) {
            const double oz = v.at("frame_check").at("origin_z").get<double>();
            REQUIRE(oz != 0.);
            for (const Variant &mixed : {Variant{oz, 0., 0.}, Variant{0., oz, 0.}}) {
                const nlohmann::json m = run_vector(v, mixed);
                CAPTURE(m.dump());
                REQUIRE((m.at("accepted") != got.at("accepted") ||
                         m.at("rejected") != got.at("rejected")));
            }
        }
    }
}

// Task 6.5: a place an auxiliary pillar could stand, but not be braced from
// its pillar, is skipped for the next angle that is both.
TEST_CASE("the prop search skips an angle the prop could not be braced from", "[bracing_engine]")
{
    // A small block 25 mm up with one support point under it: one tall
    // pillar, nothing to brace it to, so it gets one prop.
    SupportTreeConfig cfg;
    cfg.object_elevation_mm = 25.;
    TriangleMesh mesh = make_cube(2., 2., 2.);
    mesh.translate(-1.f, -1.f, 0.f);

    SupportPoint sp;
    sp.pos = {0.f, 0.f, 0.f};
    sp.head_front_radius = 0.4f;

    struct Run { Vec3d main, prop; double r = 0.; };
    auto run = [&](const PriorPillars &prior) {
        SupportableMesh sm{mesh.its, {sp}, cfg};
        sm.zoffset = 0.;
        sm.prior = prior;
        JobController       ctl;
        SupportTreeElements els;
        create_support_tree(sm, ctl, nullptr, nullptr, nullptr, &els);

        Run out;
        int props = 0;
        for (const TreePillar &p : els.pillars) {
            if (p.props < 0) continue;
            ++props;
            const TreePillar &m = els.pillars.at(size_t(p.props));
            out.main = m.endpt;
            out.prop = p.endpt;
            out.r    = m.r_start;
            REQUIRE(m.height > SupportTreeConfig::max_solo_pillar_height_mm);
            REQUIRE(m.height < SupportTreeConfig::max_dual_pillar_height_mm);
        }
        REQUIRE(props == 1);
        return out;
    };
    auto angle_step = [](const Run &r) {
        const Vec3d  d = r.prop - r.main;
        double a = std::atan2(d.y(), d.x());
        if (a < 0.) a += 2 * PI;
        const double k = a / (0.1 * PI); // the search steps by 0.1 pi
        REQUIRE(std::abs(k - std::round(k)) < 1e-6);
        return int(std::lround(k)) % 20;
    };

    // Alone, the prop stands at the first angle: that place can take it.
    const Run free_run = run({});
    REQUIRE(angle_step(free_run) == 0);

    // A full frozen pillar halfway to that place: the prop could still stand
    // there, but its bracing would pass through the frozen pillar.
    const Vec3d mid = 0.5 * (free_run.main + free_run.prop);
    PriorPillar obstacle;
    obstacle.id       = 900;
    obstacle.endpoint = {mid.x(), mid.y(), free_run.main.z()};
    obstacle.height   = 5.;
    obstacle.r_start  = obstacle.r_end = free_run.r;
    obstacle.links    = 3;
    obstacle.pairs    = 3;

    const Run blocked = run({obstacle});
    REQUIRE((blocked.main - free_run.main).norm() < 1e-9);

    // The first angle whose bracing keeps clear of the obstacle, by the rule
    // the planner uses.
    const double dist = (free_run.prop - free_run.main).head<2>().norm();
    const double lim  = free_run.r + obstacle.r_start + SAFETY_CLEARANCE_MM;
    int expected = -1;
    for (int k = 0; k < 20 && expected < 0; ++k) {
        const double a = k * 0.1 * PI;
        const Vec2d  m = free_run.main.head<2>();
        const Vec2d  p = m + dist * Vec2d{std::cos(a), std::sin(a)};
        if (geom::point_seg_dist2d(Vec2d{mid.x(), mid.y()}, m, p) >= lim) expected = k;
    }
    REQUIRE(expected > 0);
    CAPTURE(expected);
    REQUIRE(angle_step(blocked) == expected);
}

TEST_CASE("a route names the pillar on its last bar only", "[bracing_engine]")
{
    // A 4 mm block on the plate and a junction 12 mm up, above its edge. The
    // bracing exempts a bar from the clearance test against a group only at
    // that group's own pillars, so the bar ending on a pillar's top has to
    // name the pillar, and a bar ending anywhere else must not.
    TriangleMesh mesh = make_cube(4., 4., 4.);
    mesh.translate(-2.f, -2.f, 0.f);
    const Vec3d  jp{1.5, 0., 12.};
    const Vec3d  diag = Vec3d{1., 0., -1.}.normalized();

    auto check = [](const SupportTreeBuilder &b, long pid) {
        REQUIRE(pid >= 0);
        const Vec3d top = b.pillar(pid).startpoint();
        size_t owned = 0;
        for (const Bridge &br : b.bridges()) {
            REQUIRE(br.kind == BridgeKind::route);
            REQUIRE(br.owner_a == SupportTreeNode::ID_UNSET);
            const bool on_top = (br.endp - top).norm() < 1e-9;
            REQUIRE(on_top == (br.owner_b == pid));
            owned += br.owner_b == pid;
        }
        REQUIRE(owned == 1);
        REQUIRE(b.diffbridges().empty());
    };

    SECTION("on the plate, the corrector bar to a clear floor") {
        SupportTreeConfig cfg;
        cfg.object_elevation_mm = 0.;
        SupportableMesh sm{mesh.its, {}, cfg};
        SupportTreeBuilder b;
        bool laid = false;
        const auto [ok, pid] = create_ground_pillar(ex_seq, b, sm, jp, DOWN,
                                                    cfg.head_back_radius_mm,
                                                    cfg.head_back_radius_mm,
                                                    SupportTreeNode::ID_UNSET,
                                                    SupportTreeNode::ID_UNSET, &laid);
        REQUIRE(ok);
        REQUIRE(laid);
        REQUIRE(b.bridges().size() == 1);
        check(b, pid);
    }

    SECTION("on the plate, a route whose end still needs a corrector") {
        SupportTreeConfig cfg;
        cfg.object_elevation_mm = 0.;
        SupportableMesh sm{mesh.its, {}, cfg};
        SupportTreeBuilder b;
        const auto [ok, pid] = connect_to_ground(ex_seq, b, sm,
                                                 Junction{jp, cfg.head_back_radius_mm},
                                                 diag, cfg.head_back_radius_mm);
        REQUIRE(ok);
        REQUIRE(b.bridges().size() == 2); // the route, then the corrector
        check(b, pid);
    }

    SECTION("elevated, a route straight onto the pillar") {
        SupportTreeConfig cfg;
        cfg.object_elevation_mm = 5.;
        SupportableMesh sm{mesh.its, {}, cfg};
        SupportTreeBuilder b;
        const auto [ok, pid] = connect_to_ground(ex_seq, b, sm,
                                                 Junction{jp, cfg.head_back_radius_mm},
                                                 diag, cfg.head_back_radius_mm);
        REQUIRE(ok);
        REQUIRE(b.bridges().size() == 1);
        check(b, pid);
    }
}

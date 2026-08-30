#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>

#include "libslic3r/SLA/SupportPointIO.hpp"

using namespace Slic3r;

// Support point interchange format coverage (openspec change
// per-point-support-sizing, tasks 5.4 and 5.5).
//
// The format has two asymmetric halves and the tests below exist mostly to
// keep them from drifting into each other:
//
//   * writing FREEZES every size to a concrete number, so an exported file can
//     never contain the -1 sentinel;
//   * reading turns a MISSING size back into the sentinel, so the point falls
//     back to the global setting at build time.
//
// The exception to the second rule - head_front_radius, where -1 means "delete
// this point" rather than "unset" - has a test of its own.

namespace {

// Deliberately not the library defaults, so a test that accidentally reads the
// wrong config still fails.
sla::SupportTreeConfig test_cfg()
{
    sla::SupportTreeConfig cfg;
    cfg.head_front_radius_mm = 0.3;
    cfg.head_back_radius_mm  = 0.7;
    cfg.head_width_mm        = 1.4;
    cfg.head_penetration_mm  = 0.6;
    cfg.base_radius_mm       = 2.5;
    cfg.bridge_slope         = 45. * sla::support_point_deg_to_rad;
    return cfg;
}

// A point with every extension field left unset.
sla::SupportPoint bare_point(const Vec3f &at = Vec3f(1.f, 2.f, 3.f))
{
    sla::SupportPoint sp;
    sp.pos = at;
    sp.head_front_radius = 0.3f;
    return sp;
}

// A point with every extension field carrying its own value.
sla::SupportPoint fully_custom_point()
{
    sla::SupportPoint sp = bare_point(Vec3f(-4.5f, 6.25f, 11.125f));
    sp.type                      = sla::SupportPointType::slope;
    sp.head_front_radius         = 0.45f;
    sp.head_back_radius_mm       = 0.9f;
    sp.head_width_mm             = 1.75f;
    sp.head_penetration_mm       = 0.35f;
    sp.contact_sphere_radius     = 1.25f;
    sp.base_radius_mm            = 3.5f;
    sp.support_bracing_angle_deg = 62.f;
    return sp;
}

sla::ModelFingerprint some_fingerprint()
{
    sla::ModelFingerprint fp;
    fp.face_count      = 12345;
    fp.bbox_min        = {{-50000, -50000, 0}};
    fp.bbox_max        = {{50000, 50000, 200000}};
    fp.vertex_checksum = 0xfeedfacecafebeefull;
    return fp;
}

// Round trip through the actual text, not just the json object, so a
// serialization problem cannot hide.
sla::SupportPointFile round_trip(const sla::SupportPoints &pts,
                                 const sla::SupportTreeConfig &cfg)
{
    const std::string text =
        sla::support_points_to_string(pts, some_fingerprint(), cfg);

    sla::SupportPointFile out;
    std::string err;
    REQUIRE(sla::support_points_from_string(text, cfg, out, err));
    REQUIRE(err.empty());
    return out;
}

// Reads one point out of a hand written points array.
sla::SupportPoint read_one(const std::string &point_json,
                           const sla::SupportTreeConfig &cfg)
{
    const std::string text = "{ \"version\": 1, \"points\": [" + point_json + "] }";

    sla::SupportPointFile out;
    std::string err;
    REQUIRE(sla::support_points_from_string(text, cfg, out, err));
    REQUIRE(out.points.size() == 1);
    return out.points.front();
}

} // namespace

// ---------------------------------------------------------------------------
// Task 5.4
// ---------------------------------------------------------------------------

TEST_CASE("A fully specified point survives a JSON round trip",
          "[sla_support_point_io]")
{
    const sla::SupportTreeConfig cfg = test_cfg();
    const sla::SupportPoint      sp  = fully_custom_point();

    const sla::SupportPointFile back = round_trip({sp}, cfg);

    REQUIRE(back.version == sla::support_point_format_version);
    REQUIRE(back.points.size() == 1);

    const sla::SupportPoint &r = back.points.front();

    // Field by field rather than operator==, so a failure names the field.
    REQUIRE(r.pos.x() == sp.pos.x());
    REQUIRE(r.pos.y() == sp.pos.y());
    REQUIRE(r.pos.z() == sp.pos.z());
    REQUIRE(r.type == sp.type);
    REQUIRE(r.head_front_radius == sp.head_front_radius);
    REQUIRE(r.head_back_radius_mm == sp.head_back_radius_mm);
    REQUIRE(r.head_width_mm == sp.head_width_mm);
    REQUIRE(r.head_penetration_mm == sp.head_penetration_mm);
    REQUIRE(r.contact_sphere_radius == sp.contact_sphere_radius);
    REQUIRE(r.base_radius_mm == sp.base_radius_mm);
    REQUIRE(r.support_bracing_angle_deg == sp.support_bracing_angle_deg);

    REQUIRE(r == sp);
}

TEST_CASE("The fingerprint survives the round trip", "[sla_support_point_io]")
{
    const sla::SupportPointFile back = round_trip({bare_point()}, test_cfg());

    REQUIRE(back.has_fingerprint);
    REQUIRE(back.fingerprint == some_fingerprint());
}

TEST_CASE("All three type strings round trip unchanged", "[sla_support_point_io]")
{
    const sla::SupportTreeConfig cfg = test_cfg();

    sla::SupportPoints pts;
    for (sla::SupportPointType t : {sla::SupportPointType::manual_add,
                                    sla::SupportPointType::island,
                                    sla::SupportPointType::slope}) {
        sla::SupportPoint sp = bare_point();
        sp.type = t;
        pts.push_back(sp);
    }

    // The strings themselves are part of the contract, so pin them.
    const nlohmann::json j = sla::support_points_to_json(pts, some_fingerprint(), cfg);
    REQUIRE(j[sla::spkey::points][0][sla::spkey::type] == "manual_add");
    REQUIRE(j[sla::spkey::points][1][sla::spkey::type] == "island");
    REQUIRE(j[sla::spkey::points][2][sla::spkey::type] == "slope");

    const sla::SupportPointFile back = round_trip(pts, cfg);
    REQUIRE(back.points.size() == 3);
    REQUIRE(back.points[0].type == sla::SupportPointType::manual_add);
    REQUIRE(back.points[1].type == sla::SupportPointType::island);
    REQUIRE(back.points[2].type == sla::SupportPointType::slope);
}

TEST_CASE("An unset point is exported frozen, with no sentinel",
          "[sla_support_point_io]")
{
    const sla::SupportTreeConfig cfg = test_cfg();

    const nlohmann::json j =
        sla::support_points_to_json({bare_point()}, some_fingerprint(), cfg);
    const nlohmann::json &p = j[sla::spkey::points][0];

    // head_front_radius is not part of the sentinel scheme, so this point
    // carries its own real value and it is written through unchanged. The
    // widening is what a float member looks like as a double.
    REQUIRE(p[sla::spkey::head_front_radius].get<double>() == double(0.3f));

    // The rest are the resolved global values, spelled out.
    REQUIRE(p[sla::spkey::head_back_radius_mm].get<double>() == cfg.head_back_radius_mm);
    REQUIRE(p[sla::spkey::head_width_mm].get<double>() == cfg.head_width_mm);
    REQUIRE(p[sla::spkey::head_penetration_mm].get<double>() == cfg.head_penetration_mm);
    REQUIRE(p[sla::spkey::base_radius_mm].get<double>() == cfg.base_radius_mm);
    // Degrees out, radians in the config, so this one is a conversion.
    REQUIRE(p[sla::spkey::support_bracing_angle_deg].get<double>() ==
            Catch::Approx(45.));
    // No global contact sphere setting exists in this fork, so an unset field
    // freezes as 0 - the tri-state value for "no contact sphere on this point".
    REQUIRE(p[sla::spkey::contact_sphere_radius].get<double>() == 0.);

    // And the blanket rule: nothing negative anywhere in the file.
    for (const nlohmann::json &pt : j[sla::spkey::points])
        for (auto it = pt.begin(); it != pt.end(); ++it)
            if (it->is_number())
                REQUIRE(it->get<double>() >= 0.);
}

TEST_CASE("A head_front_radius delete marker is never written out",
          "[sla_support_point_io]")
{
    // -1 in this field means "marked for erase" to
    // prepare_permanent_support_points(). Writing it into a file would produce
    // points that delete themselves the next time they are imported, so the
    // writer resolves it to the global value instead.
    const sla::SupportTreeConfig cfg = test_cfg();

    sla::SupportPoint sp = bare_point();
    sp.head_front_radius = -1.f;

    const nlohmann::json j =
        sla::support_points_to_json({sp}, some_fingerprint(), cfg);

    REQUIRE(j[sla::spkey::points][0][sla::spkey::head_front_radius].get<double>() ==
            cfg.head_front_radius_mm);
}

TEST_CASE("Freezing ignores the global config that comes later",
          "[sla_support_point_io]")
{
    // The point of freezing: a file exported under one pillar radius must
    // still describe that radius after the global setting moves.
    sla::SupportTreeConfig at_export = test_cfg();
    at_export.head_back_radius_mm = 1.0;

    const std::string text =
        sla::support_points_to_string({bare_point()}, some_fingerprint(), at_export);

    sla::SupportTreeConfig at_import = test_cfg();
    at_import.head_back_radius_mm = 2.0;

    sla::SupportPointFile out;
    std::string err;
    REQUIRE(sla::support_points_from_string(text, at_import, out, err));
    REQUIRE(out.points.front().head_back_radius_mm == 1.0f);
}

TEST_CASE("The export never contains the UI only keys", "[sla_support_point_io]")
{
    const nlohmann::json j = sla::support_points_to_json(
        {fully_custom_point(), bare_point()}, some_fingerprint(), test_cfg());

    for (const nlohmann::json &pt : j[sla::spkey::points]) {
        REQUIRE(pt.find("pillar_radius") == pt.end());
        REQUIRE(pt.find("weight") == pt.end());
    }

    // Not anywhere else in the document either.
    const std::string text = j.dump();
    REQUIRE(text.find("pillar_radius") == std::string::npos);
    REQUIRE(text.find("weight") == std::string::npos);
}

TEST_CASE("An explicit zero contact sphere is not folded into unset",
          "[sla_support_point_io]")
{
    const sla::SupportTreeConfig cfg = test_cfg();

    sla::SupportPoint sp = bare_point();
    sp.contact_sphere_radius = 0.f; // "this point explicitly has none"

    const sla::SupportPointFile back = round_trip({sp}, cfg);
    REQUIRE(back.points.front().contact_sphere_radius == 0.f);
    REQUIRE(sla::point_contact_sphere_state(back.points.front()) ==
            sla::ContactSphereState::Disabled);
}

// ---------------------------------------------------------------------------
// Task 5.5
// ---------------------------------------------------------------------------

TEST_CASE("A point with nothing but pos falls back to the global defaults",
          "[sla_support_point_io]")
{
    const sla::SupportTreeConfig cfg = test_cfg();
    const sla::SupportPoint sp = read_one("{ \"pos\": [1.5, 2.5, 3.5] }", cfg);

    REQUIRE(sp.pos.x() == 1.5f);
    REQUIRE(sp.pos.y() == 2.5f);
    REQUIRE(sp.pos.z() == 3.5f);

    // An absent type means a hand added point.
    REQUIRE(sp.type == sla::SupportPointType::manual_add);

    // All six extension fields sit at the sentinel, so the resolvers hand back
    // the global setting.
    REQUIRE(sp.head_back_radius_mm == sla::support_point_unset);
    REQUIRE(sp.head_width_mm == sla::support_point_unset);
    REQUIRE(sp.head_penetration_mm == sla::support_point_unset);
    REQUIRE(sp.contact_sphere_radius == sla::support_point_unset);
    REQUIRE(sp.base_radius_mm == sla::support_point_unset);
    REQUIRE(sp.support_bracing_angle_deg == sla::support_point_unset);

    REQUIRE(sla::point_head_back_radius_mm(sp, cfg.head_back_radius_mm) ==
            cfg.head_back_radius_mm);
    REQUIRE(sla::point_base_radius_mm(sp, cfg.base_radius_mm) == cfg.base_radius_mm);
    REQUIRE(sla::point_bracing_angle_rad(sp, cfg.bridge_slope) == cfg.bridge_slope);

    // head_front_radius is the field that must NOT get the sentinel: -1 there
    // means "marked for erase" and the point would be dropped silently by
    // prepare_permanent_support_points().
    REQUIRE(sp.head_front_radius >= 0.f);
    REQUIRE(sp.head_front_radius == float(cfg.head_front_radius_mm));
}

TEST_CASE("An explicit -1 head_front_radius is not stored as -1",
          "[sla_support_point_io]")
{
    // Even if a caller writes the delete marker into the file, importing must
    // not carry it into the model.
    const sla::SupportTreeConfig cfg = test_cfg();
    const sla::SupportPoint sp =
        read_one("{ \"pos\": [0,0,0], \"head_front_radius\": -1 }", cfg);

    REQUIRE(sp.head_front_radius == float(cfg.head_front_radius_mm));
}

TEST_CASE("A head_front_radius of zero is refused as a stored value",
          "[sla_support_point_io]")
{
    // Zero is the quiet version of the -1 delete marker.
    // prepare_permanent_support_points() (SLAPrintSteps.cpp) drops a point when
    // "dist_sq >= sqr(head_front_radius)". A squared distance is never
    // negative, so a radius of 0 makes that true for every point in existence,
    // and the point disappears with only an assert that release builds compile
    // away. It must never be what gets stored.
    const sla::SupportTreeConfig cfg = test_cfg();
    const sla::SupportPoint sp =
        read_one("{ \"pos\": [0,0,0], \"head_front_radius\": 0 }", cfg);

    REQUIRE(sp.head_front_radius > 0.f);
    REQUIRE(sp.head_front_radius == float(cfg.head_front_radius_mm));
}

TEST_CASE("A zero head_front_radius is not written out either",
          "[sla_support_point_io]")
{
    const sla::SupportTreeConfig cfg = test_cfg();

    sla::SupportPoint sp = bare_point();
    sp.head_front_radius = 0.f;

    const nlohmann::json j =
        sla::support_points_to_json({sp}, some_fingerprint(), cfg);

    REQUIRE(j[sla::spkey::points][0][sla::spkey::head_front_radius].get<double>() ==
            cfg.head_front_radius_mm);
}

TEST_CASE("A config with no usable head radius fails loudly",
          "[sla_support_point_io]")
{
    // If the fallback itself is unusable there is nothing to fall back TO.
    // Saying so beats building points that will be silently thrown away later.
    sla::SupportTreeConfig broken = test_cfg();
    broken.head_front_radius_mm = 0.;

    sla::SupportPointFile out;
    std::string err;
    REQUIRE_FALSE(sla::support_points_from_string(
        "{ \"version\": 1, \"points\": [ { \"pos\": [0,0,0] } ] }",
        broken, out, err));
    REQUIRE_FALSE(err.empty());

    broken.head_front_radius_mm = -0.5;
    REQUIRE_FALSE(sla::support_points_from_string(
        "{ \"version\": 1, \"points\": [ { \"pos\": [0,0,0] } ] }",
        broken, out, err));
}

TEST_CASE("Values a float cannot hold are refused, not narrowed",
          "[sla_support_point_io]")
{
    // Narrowing a double that is outside float's range is UNDEFINED behaviour,
    // not merely a lossy conversion, so it has to be caught before the cast.
    // 1e300 is finite - isfinite() alone would wave it through.
    const sla::SupportTreeConfig cfg = test_cfg();
    sla::SupportPointFile out;
    std::string err;

    auto refused = [&](const char *text) {
        const bool ok = sla::support_points_from_string(text, cfg, out, err);
        return !ok && !err.empty();
    };

    REQUIRE(refused("{ \"version\": 1, \"points\": "
                    "[ { \"pos\": [1e300, 0, 0] } ] }"));
    REQUIRE(refused("{ \"version\": 1, \"points\": "
                    "[ { \"pos\": [0, 0, -1e300] } ] }"));
    REQUIRE(refused("{ \"version\": 1, \"points\": "
                    "[ { \"pos\": [0,0,0], \"base_radius_mm\": 1e300 } ] }"));
    REQUIRE(refused("{ \"version\": 1, \"points\": "
                    "[ { \"pos\": [0,0,0], \"head_front_radius\": 1e300 } ] }"));
    REQUIRE(refused("{ \"version\": 1, \"points\": "
                    "[ { \"pos\": [0,0,0], \"head_width_mm\": -1e300 } ] }"));

    // Just inside the range is still accepted.
    err.clear();
    REQUIRE(sla::support_points_from_string(
        "{ \"version\": 1, \"points\": "
        "[ { \"pos\": [0,0,0], \"base_radius_mm\": 1e30 } ] }", cfg, out, err));
}

TEST_CASE("A broken config cannot push a negative number into the export",
          "[sla_support_point_io]")
{
    // The globals come straight from the config with no clamping of their own
    // (SLAPrint.cpp: make_support_cfg), so the writer has to be the one that
    // guarantees a non-negative file.
    sla::SupportTreeConfig broken = test_cfg();
    broken.head_back_radius_mm = -1.0;
    broken.head_width_mm       = -2.0;
    broken.head_penetration_mm = -0.5;
    broken.base_radius_mm      = -3.0;
    broken.bridge_slope        = -1.0;

    const nlohmann::json j =
        sla::support_points_to_json({bare_point()}, some_fingerprint(), broken);

    for (const nlohmann::json &pt : j[sla::spkey::points])
        for (auto it = pt.begin(); it != pt.end(); ++it)
            if (it->is_number())
                REQUIRE(it->get<double>() >= 0.);
}

TEST_CASE("A stale error message is not left behind on success",
          "[sla_support_point_io]")
{
    const sla::SupportTreeConfig cfg = test_cfg();
    sla::SupportPointFile out;
    std::string err = "left over from an earlier call";

    REQUIRE(sla::support_points_from_string(
        "{ \"version\": 1, \"points\": [ { \"pos\": [0,0,0] } ] }",
        cfg, out, err));
    REQUIRE(err.empty());
}

TEST_CASE("A partially specified point keeps only what it was given",
          "[sla_support_point_io]")
{
    const sla::SupportTreeConfig cfg = test_cfg();
    const sla::SupportPoint sp = read_one(
        "{ \"pos\": [0,0,0], \"head_back_radius_mm\": 1.25 }", cfg);

    REQUIRE(sp.head_back_radius_mm == 1.25f);
    REQUIRE(sla::point_head_back_radius_mm(sp, cfg.head_back_radius_mm) == 1.25);

    // Everything else still falls back.
    REQUIRE(sp.head_width_mm == sla::support_point_unset);
    REQUIRE(sp.head_penetration_mm == sla::support_point_unset);
    REQUIRE(sp.base_radius_mm == sla::support_point_unset);
    REQUIRE(sp.support_bracing_angle_deg == sla::support_point_unset);
    REQUIRE(sla::point_head_width_mm(sp, cfg.head_width_mm) == cfg.head_width_mm);
}

TEST_CASE("A negative extension size normalises to the sentinel",
          "[sla_support_point_io]")
{
    const sla::SupportTreeConfig cfg = test_cfg();
    const sla::SupportPoint sp = read_one(
        "{ \"pos\": [0,0,0], \"base_radius_mm\": -1, \"head_width_mm\": -7.5 }", cfg);

    REQUIRE(sp.base_radius_mm == sla::support_point_unset);
    REQUIRE(sp.head_width_mm == sla::support_point_unset);
    REQUIRE(sla::point_base_radius_mm(sp, cfg.base_radius_mm) == cfg.base_radius_mm);
}

TEST_CASE("Unknown keys are ignored", "[sla_support_point_io]")
{
    const sla::SupportTreeConfig cfg = test_cfg();

    // Unknown keys on the point, including the two that are banned from the
    // format, plus an unknown key at the top level.
    const std::string text =
        "{ \"version\": 1, \"future_top_level_key\": 42, \"points\": ["
        "  { \"pos\": [1,2,3], \"type\": \"island\","
        "    \"head_back_radius_mm\": 0.8,"
        "    \"pillar_radius\": 9.9, \"weight\": 3, \"something_new\": true } ] }";

    sla::SupportPointFile out;
    std::string err;
    REQUIRE(sla::support_points_from_string(text, cfg, out, err));
    REQUIRE(err.empty());
    REQUIRE(out.points.size() == 1);

    // The known fields still landed.
    REQUIRE(out.points.front().type == sla::SupportPointType::island);
    REQUIRE(out.points.front().head_back_radius_mm == 0.8f);
    REQUIRE(out.points.front().pos.z() == 3.f);
}

TEST_CASE("An unrecognised version is refused", "[sla_support_point_io]")
{
    const sla::SupportTreeConfig cfg = test_cfg();
    sla::SupportPointFile out;
    std::string err;

    // A future version, whose keys may not mean what they appear to mean.
    REQUIRE_FALSE(sla::support_points_from_string(
        "{ \"version\": 2, \"points\": [] }", cfg, out, err));
    REQUIRE_FALSE(err.empty());

    // A version that never existed.
    err.clear();
    REQUIRE_FALSE(sla::support_points_from_string(
        "{ \"version\": 0, \"points\": [] }", cfg, out, err));
    REQUIRE_FALSE(err.empty());

    // No version at all.
    err.clear();
    REQUIRE_FALSE(sla::support_points_from_string(
        "{ \"points\": [] }", cfg, out, err));
    REQUIRE_FALSE(err.empty());

    // A version of the wrong type is refused rather than coerced.
    err.clear();
    REQUIRE_FALSE(sla::support_points_from_string(
        "{ \"version\": \"1\", \"points\": [] }", cfg, out, err));
    REQUIRE_FALSE(err.empty());
}

TEST_CASE("Malformed documents are refused rather than half read",
          "[sla_support_point_io]")
{
    const sla::SupportTreeConfig cfg = test_cfg();
    sla::SupportPointFile out;
    std::string err;

    auto refused = [&](const char *text) {
        err.clear();
        const bool ok = sla::support_points_from_string(text, cfg, out, err);
        return !ok && !err.empty();
    };

    REQUIRE(refused("not json at all"));
    REQUIRE(refused("[]"));
    REQUIRE(refused("{ \"version\": 1 }"));                     // no points
    REQUIRE(refused("{ \"version\": 1, \"points\": {} }"));      // points not an array
    REQUIRE(refused("{ \"version\": 1, \"points\": [ 7 ] }"));   // point not an object
    REQUIRE(refused("{ \"version\": 1, \"points\": [ {} ] }"));  // no pos
    REQUIRE(refused("{ \"version\": 1, \"points\": [ { \"pos\": [1,2] } ] }"));
    REQUIRE(refused("{ \"version\": 1, \"points\": [ { \"pos\": [1,2,\"x\"] } ] }"));
    // An unknown type string must not be guessed at.
    REQUIRE(refused("{ \"version\": 1, \"points\": "
                    "[ { \"pos\": [0,0,0], \"type\": \"thin\" } ] }"));
    // A size of the wrong type is an error, not a silent fallback.
    REQUIRE(refused("{ \"version\": 1, \"points\": "
                    "[ { \"pos\": [0,0,0], \"base_radius_mm\": \"2\" } ] }"));
    // A broken fingerprint block fails the whole file.
    REQUIRE(refused("{ \"version\": 1, \"points\": [], "
                    "\"model_fingerprint\": { \"face_count\": -1 } }"));
    REQUIRE(refused("{ \"version\": 1, \"points\": [], \"model_fingerprint\": "
                    "{ \"face_count\": 1, \"bbox_min\": [0,0,0], "
                    "\"bbox_max\": [1,1,1], \"vertex_checksum\": \"abc\" } }"));
}

TEST_CASE("A file with no fingerprint block still loads",
          "[sla_support_point_io]")
{
    // Whether a missing fingerprint is acceptable is the import path's call,
    // not the parser's. The parser reports its absence rather than inventing
    // one.
    const sla::SupportTreeConfig cfg = test_cfg();
    sla::SupportPointFile out;
    std::string err;

    REQUIRE(sla::support_points_from_string(
        "{ \"version\": 1, \"points\": [ { \"pos\": [0,0,0] } ] }", cfg, out, err));
    REQUIRE_FALSE(out.has_fingerprint);
    REQUIRE(out.points.size() == 1);
}

TEST_CASE("An empty point list is a valid document", "[sla_support_point_io]")
{
    const sla::SupportTreeConfig cfg = test_cfg();
    const sla::SupportPointFile back = round_trip({}, cfg);
    REQUIRE(back.points.empty());
    REQUIRE(back.has_fingerprint);
}

TEST_CASE("Repeated round trips reach a fixed point", "[sla_support_point_io]")
{
    // Handing the same file back and forth must not make it drift. It settles
    // one step later than one might expect, and the reason is worth spelling
    // out: a size taken from the config is a double, while the same size held
    // on a SupportPoint is a float. So a size that fell back to the global
    // setting is written at double precision the first time and at float
    // precision every time after, and the two texts differ in the last digits.
    //
    // What is guaranteed, and what this pins down:
    //   - the POINTS are a fixed point from the first import onwards;
    //   - the TEXT is a fixed point from the second export onwards.
    const sla::SupportTreeConfig cfg = test_cfg();
    const sla::SupportPoints pts = {fully_custom_point(), bare_point()};

    const std::string t1 = sla::support_points_to_string(pts, some_fingerprint(), cfg);

    sla::SupportPointFile f1;
    std::string err;
    REQUIRE(sla::support_points_from_string(t1, cfg, f1, err));

    const std::string t2 = sla::support_points_to_string(f1.points, f1.fingerprint, cfg);

    sla::SupportPointFile f2;
    REQUIRE(sla::support_points_from_string(t2, cfg, f2, err));

    const std::string t3 = sla::support_points_to_string(f2.points, f2.fingerprint, cfg);

    REQUIRE(t2 == t3);

    REQUIRE(f1.points.size() == f2.points.size());
    for (size_t i = 0; i < f1.points.size(); ++i) {
        // Bit for bit, not the tolerant operator==.
        REQUIRE(f1.points[i].pos == f2.points[i].pos);
        REQUIRE(f1.points[i].type == f2.points[i].type);
        REQUIRE(f1.points[i].head_front_radius == f2.points[i].head_front_radius);
        REQUIRE(f1.points[i].head_back_radius_mm == f2.points[i].head_back_radius_mm);
        REQUIRE(f1.points[i].head_width_mm == f2.points[i].head_width_mm);
        REQUIRE(f1.points[i].head_penetration_mm == f2.points[i].head_penetration_mm);
        REQUIRE(f1.points[i].contact_sphere_radius == f2.points[i].contact_sphere_radius);
        REQUIRE(f1.points[i].base_radius_mm == f2.points[i].base_radius_mm);
        REQUIRE(f1.points[i].support_bracing_angle_deg ==
                f2.points[i].support_bracing_angle_deg);
    }

    // And the difference between t1 and t2 really is only precision: reading
    // either one gives the very same points.
    REQUIRE(f1.points.size() == pts.size());
}

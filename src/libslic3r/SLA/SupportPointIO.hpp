///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef SLA_SUPPORTPOINTIO_HPP
#define SLA_SUPPORTPOINTIO_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

#include "nlohmann/json.hpp"

#include <libslic3r/SLA/ModelFingerprint.hpp>
#include <libslic3r/SLA/SupportPoint.hpp>
#include <libslic3r/SLA/SupportTree.hpp>

namespace Slic3r { namespace sla {

// ---------------------------------------------------------------------------
// Support point interchange format (openspec change per-point-support-sizing,
// tasks 5.1 - 5.3)
//
// A JSON document that carries a list of support points out of the engine and
// back in again. Every field is located by name; there is no fixed positional
// layout, and the reader ignores keys it does not know so a file written by a
// newer engine stays loadable.
//
// Two rules shape the whole format and are easy to get backwards:
//
//   * ON THE WAY OUT the sizes are FROZEN. Each point is written with the
//     concrete millimetre value it would actually be built with right now,
//     resolved against the global config. No sentinel ever leaves here. That
//     is what makes an exported list self-describing: changing a global
//     setting between export and import cannot move points already exported.
//
//   * ON THE WAY IN a missing size key means the SENTINEL, so the point falls
//     back to whatever the global setting is at build time. That is what lets
//     a caller add a point by writing nothing but its position.
//
// The one field that does not follow the second rule is head_front_radius -
// see support_point_from_json().
//
// A note on precision. SupportPoint holds its sizes as float, while the global
// config holds them as double. A size that fell back to the global setting is
// therefore written at double precision on the first export and at float
// precision on every export after it, and the two texts differ in the last few
// digits. The POINTS are stable from the first import onwards; the TEXT is
// stable from the second export onwards. Nothing downstream depends on the
// text being byte-identical, and the difference is around 1e-8 mm.
// ---------------------------------------------------------------------------

// Printed to stderr, verbatim, when an imported point list does not describe
// the model being sliced.
//
// This string is a CONTRACT with agent/support_classifier.py, which matches on
// it to attribute the failure to SUPPORT_POINTS_MODEL_MISMATCH. It must stay a
// raw literal - never wrapped in _u8L() or I18N::translate - so that it reads
// the same in every locale. The exit code is deliberately NOT part of the
// contract: this fork returns 0 from several failing paths, so the classifier
// keys on this marker and nothing else.
inline constexpr const char *support_points_model_mismatch_marker =
    "SUPPORT_POINTS_MODEL_MISMATCH: imported support points do not match this model";

// Bumped only when the meaning of an existing key changes. Adding a key does
// not need a bump: unknown keys are ignored on read, and a missing key already
// has a defined meaning.
inline constexpr int support_point_format_version = 1;

// Every key name in the format, in one place, so no string is ever spelled out
// at a call site.
namespace spkey {

// Top level.
inline constexpr const char *version           = "version";
inline constexpr const char *model_fingerprint = "model_fingerprint";
inline constexpr const char *points            = "points";

// Inside model_fingerprint.
inline constexpr const char *face_count      = "face_count";
inline constexpr const char *bbox_min        = "bbox_min";
inline constexpr const char *bbox_max        = "bbox_max";
inline constexpr const char *vertex_checksum = "vertex_checksum";

// Inside one entry of points. These deliberately match the C++ member names.
inline constexpr const char *pos                       = "pos";
inline constexpr const char *type                      = "type";
inline constexpr const char *head_front_radius         = "head_front_radius";
inline constexpr const char *head_back_radius_mm       = "head_back_radius_mm";
inline constexpr const char *head_width_mm             = "head_width_mm";
inline constexpr const char *head_penetration_mm       = "head_penetration_mm";
inline constexpr const char *contact_sphere_radius     = "contact_sphere_radius";
inline constexpr const char *base_radius_mm            = "base_radius_mm";
inline constexpr const char *support_bracing_angle_deg = "support_bracing_angle_deg";

// Values of the type key.
inline constexpr const char *type_manual_add = "manual_add";
inline constexpr const char *type_island     = "island";
inline constexpr const char *type_slope      = "slope";

// NOTE: pillar_radius and weight are absent on purpose and must stay absent.
// The engine reads neither - the pillar radius comes from head_back_radius_mm,
// and weight is a desktop UI highlight. See decision D7.

} // namespace spkey

// ---------------------------------------------------------------------------
// SupportPointType <-> string
//
// A string, not the 3MF trick of comparing a float against 1.0 within a
// tolerance. An unrecognised string is an error, never a guess.
// ---------------------------------------------------------------------------

inline const char *support_point_type_to_string(SupportPointType t)
{
    switch (t) {
    case SupportPointType::island: return spkey::type_island;
    case SupportPointType::slope:  return spkey::type_slope;
    case SupportPointType::manual_add:
    default:                       return spkey::type_manual_add;
    }
}

inline bool support_point_type_from_string(const std::string &s, SupportPointType &out)
{
    if (s == spkey::type_manual_add) { out = SupportPointType::manual_add; return true; }
    if (s == spkey::type_island)     { out = SupportPointType::island;     return true; }
    if (s == spkey::type_slope)      { out = SupportPointType::slope;      return true; }
    return false;
}

// ---------------------------------------------------------------------------
// Reading helpers
// ---------------------------------------------------------------------------

enum class FieldRead {
    Absent,  // key not present - has a defined meaning, not an error
    Ok,      // present and usable
    Invalid  // present but the wrong type, or not finite - an error
};

namespace detail {

// May this double be narrowed to a float?
//
// Every number read here ends up in a float member, and [conv.double] leaves
// the narrowing UNDEFINED - not merely unspecified - once the source is
// outside the destination's range. isfinite() is not enough on its own: 1e300
// is perfectly finite and still 260 orders of magnitude past FLT_MAX. So the
// check has to happen BEFORE the conversion, on every path.
inline bool in_float_range(double v)
{
    return std::isfinite(v) &&
           std::abs(v) <= double(std::numeric_limits<float>::max());
}

// Reads a number that will be narrowed to a float. out is left untouched
// unless the result is Ok, so the caller may preload it with a default.
inline FieldRead read_number(const nlohmann::json &j, const char *key,
                             double &out, std::string &err)
{
    const auto it = j.find(key);
    if (it == j.end())
        return FieldRead::Absent;

    if (!it->is_number()) {
        err = std::string("support point field ") + key + " is not a number";
        return FieldRead::Invalid;
    }

    const double v = it->template get<double>();
    if (!in_float_range(v)) {
        err = std::string("support point field ") + key +
              " is not a finite value a float can hold";
        return FieldRead::Invalid;
    }

    out = v;
    return FieldRead::Ok;
}

} // namespace detail

// ---------------------------------------------------------------------------
// Model fingerprint <-> JSON
// ---------------------------------------------------------------------------

inline nlohmann::json fingerprint_to_json(const ModelFingerprint &fp)
{
    nlohmann::json j;
    j[spkey::face_count] = fp.face_count;
    j[spkey::bbox_min]   = {fp.bbox_min[0], fp.bbox_min[1], fp.bbox_min[2]};
    j[spkey::bbox_max]   = {fp.bbox_max[0], fp.bbox_max[1], fp.bbox_max[2]};
    // A hex string, not a number: the checksum uses the full 64 bit range, and
    // a JSON consumer that parses numbers as IEEE doubles would round the top
    // bits away without saying so.
    j[spkey::vertex_checksum] = fingerprint_checksum_to_string(fp.vertex_checksum);
    return j;
}

inline bool fingerprint_from_json(const nlohmann::json &j, ModelFingerprint &out,
                                  std::string &err)
{
    err.clear(); // err is only meaningful on failure; never leave a stale one

    if (!j.is_object()) {
        err = "model_fingerprint is not an object";
        return false;
    }

    ModelFingerprint fp;

    const auto fc = j.find(spkey::face_count);
    if (fc == j.end() || !fc->is_number_integer()) {
        err = "model_fingerprint.face_count is missing or not an integer";
        return false;
    }
    const int64_t faces = fc->template get<int64_t>();
    if (faces < 0) {
        err = "model_fingerprint.face_count is negative";
        return false;
    }
    fp.face_count = static_cast<uint64_t>(faces);

    auto read_bbox = [&](const char *key, std::array<int64_t, 3> &dst) {
        const auto it = j.find(key);
        if (it == j.end() || !it->is_array() || it->size() != 3) {
            err = std::string("model_fingerprint.") + key +
                  " is missing or not an array of three integers";
            return false;
        }
        for (int k = 0; k < 3; ++k) {
            const nlohmann::json &c = (*it)[size_t(k)];
            if (!c.is_number_integer()) {
                err = std::string("model_fingerprint.") + key +
                      " contains a non-integer";
                return false;
            }
            dst[size_t(k)] = c.template get<int64_t>();
        }
        return true;
    };

    if (!read_bbox(spkey::bbox_min, fp.bbox_min)) return false;
    if (!read_bbox(spkey::bbox_max, fp.bbox_max)) return false;

    const auto cs = j.find(spkey::vertex_checksum);
    if (cs == j.end() || !cs->is_string()) {
        err = "model_fingerprint.vertex_checksum is missing or not a string";
        return false;
    }
    if (!fingerprint_checksum_from_string(cs->template get<std::string>(),
                                          fp.vertex_checksum)) {
        err = "model_fingerprint.vertex_checksum is not 16 hex digits";
        return false;
    }

    out = fp;
    return true;
}

// ---------------------------------------------------------------------------
// One support point <-> JSON
// ---------------------------------------------------------------------------

// Writes a point with every size FROZEN to the value it resolves to under cfg.
// Nothing negative is ever produced, so an exported file never contains the
// sentinel.
inline nlohmann::json support_point_to_json(const SupportPoint &sp,
                                            const SupportTreeConfig &cfg)
{
    nlohmann::json j;

    j[spkey::pos]  = {double(sp.pos.x()), double(sp.pos.y()), double(sp.pos.z())};
    j[spkey::type] = support_point_type_to_string(sp.type);

    // head_front_radius is guarded on ">0", not ">=0", and the two rejected
    // values fail for different reasons:
    //
    //   -1  means "marked for erase" in prepare_permanent_support_points(), so
    //       writing it produces a file whose points delete themselves.
    //    0  is worse, because it looks harmless. That function's first test is
    //       "dist_sq >= sqr(head_front_radius)" (SLAPrintSteps.cpp), and with a
    //       radius of 0 that reads "squared distance >= 0", which is true for
    //       every point in existence. The point is skipped, silently, with only
    //       an assert that release builds compile away.
    j[spkey::head_front_radius] = sp.head_front_radius > 0.f
                                      ? double(sp.head_front_radius)
                                      : std::max(0., cfg.head_front_radius_mm);

    // Every size is clamped here rather than trusted. The globals come straight
    // from the config (SLAPrint.cpp: make_support_cfg) with no clamping of
    // their own, so "an export never contains a negative number" has to be
    // this function's guarantee, not an assumption about its caller.
    j[spkey::head_back_radius_mm] =
        std::max(0., point_head_back_radius_mm(sp, cfg.head_back_radius_mm));
    j[spkey::head_width_mm] =
        std::max(0., point_head_width_mm(sp, cfg.head_width_mm));
    j[spkey::head_penetration_mm] =
        std::max(0., point_head_penetration_mm(sp, cfg.head_penetration_mm));

    // contact_sphere_radius has no global setting in this fork to resolve
    // against - there is no contact sphere geometry here at all (D7). An unset
    // field is therefore frozen as 0, which is the tri-state value meaning
    // "this point has no contact sphere": truthful about what will actually be
    // built, non-negative as the format requires, and stable from the second
    // export onwards because 0 reads back as the real value 0.
    j[spkey::contact_sphere_radius] =
        point_field_is_set(sp.contact_sphere_radius)
            ? std::max(0., double(sp.contact_sphere_radius))
            : 0.;

    j[spkey::base_radius_mm] =
        std::max(0., point_base_radius_mm(sp, cfg.base_radius_mm));

    // Stored in degrees; cfg.bridge_slope is radians.
    j[spkey::support_bracing_angle_deg] = std::max(
        0., point_bracing_angle_rad(sp, cfg.bridge_slope) / support_point_deg_to_rad);

    return j;
}

// Reads a point. Only pos is required.
inline bool support_point_from_json(const nlohmann::json &j,
                                    const SupportTreeConfig &cfg,
                                    SupportPoint &out, std::string &err)
{
    err.clear(); // err is only meaningful on failure; never leave a stale one

    if (!j.is_object()) {
        err = "a support point entry is not an object";
        return false;
    }

    // Starts with every extension field already at the sentinel, which is
    // exactly what an absent key has to mean.
    SupportPoint sp;

    const auto pit = j.find(spkey::pos);
    if (pit == j.end() || !pit->is_array() || pit->size() != 3) {
        err = "a support point has no pos array of three numbers";
        return false;
    }
    for (int k = 0; k < 3; ++k) {
        const nlohmann::json &c = (*pit)[size_t(k)];
        if (!c.is_number()) {
            err = "a support point pos contains a non-number";
            return false;
        }
        const double v = c.template get<double>();
        if (!detail::in_float_range(v)) {
            err = "a support point pos contains a coordinate a float cannot hold";
            return false;
        }
        sp.pos[k] = float(v);
    }

    // An absent type means manual_add: a caller adding a point by hand should
    // not have to say so. An unrecognised string is an error - guessing is
    // exactly what the format forbids.
    const auto tit = j.find(spkey::type);
    if (tit != j.end()) {
        if (!tit->is_string()) {
            err = "a support point type is not a string";
            return false;
        }
        const std::string ts = tit->template get<std::string>();
        if (!support_point_type_from_string(ts, sp.type)) {
            err = "unknown support point type " + ts;
            return false;
        }
    }

    // head_front_radius is NOT part of the sentinel scheme. An absent key -
    // and equally any NON-POSITIVE one - resolves to the concrete global value
    // right here.
    //
    // The test is "<= 0", not "< 0", and the zero case is the dangerous one.
    // prepare_permanent_support_points() (SLAPrintSteps.cpp) drops a point when
    // "dist_sq >= sqr(head_front_radius)"; with a radius of 0 that condition is
    // true for every point that exists, so the point disappears with nothing
    // but an assert that release builds compile away. -1 is the same story by a
    // more obvious route: it is that function's explicit erase marker.
    {
        double hfr = cfg.head_front_radius_mm;
        switch (detail::read_number(j, spkey::head_front_radius, hfr, err)) {
        case FieldRead::Invalid: return false;
        case FieldRead::Absent:  break;
        case FieldRead::Ok:      if (hfr <= 0.) hfr = cfg.head_front_radius_mm; break;
        }

        // The fallback is only a fallback if it is itself usable. A config with
        // a non-positive head front radius has nothing to fall back TO, so say
        // so instead of quietly building points that will be thrown away.
        if (!detail::in_float_range(hfr) || hfr <= 0.) {
            err = "head_front_radius resolves to a value that is not a usable "
                  "radius; the global support head front radius is non-positive "
                  "or out of range";
            return false;
        }
        sp.head_front_radius = float(hfr);
    }

    // The six extension fields. Absent, or any negative value, normalises to
    // the sentinel so the fallback happens once, at build time.
    //
    // Zero is NOT negative and so survives as a real value. That matters for
    // contact_sphere_radius, where 0 means "this point explicitly has no
    // contact sphere" and must never be folded into "unset".
    auto read_extension = [&](const char *key, float &dst) {
        double v = 0.;
        switch (detail::read_number(j, key, v, err)) {
        case FieldRead::Invalid: return false;
        case FieldRead::Absent:  dst = support_point_unset; break;
        case FieldRead::Ok:      dst = v < 0. ? support_point_unset : float(v); break;
        }
        return true;
    };

    if (!read_extension(spkey::head_back_radius_mm,       sp.head_back_radius_mm))       return false;
    if (!read_extension(spkey::head_width_mm,             sp.head_width_mm))             return false;
    if (!read_extension(spkey::head_penetration_mm,       sp.head_penetration_mm))       return false;
    if (!read_extension(spkey::contact_sphere_radius,     sp.contact_sphere_radius))     return false;
    if (!read_extension(spkey::base_radius_mm,            sp.base_radius_mm))            return false;
    if (!read_extension(spkey::support_bracing_angle_deg, sp.support_bracing_angle_deg)) return false;

    // Any key not named above is ignored, which is what keeps a file written
    // by a newer engine loadable here.

    out = sp;
    return true;
}

// ---------------------------------------------------------------------------
// The whole document
// ---------------------------------------------------------------------------

struct SupportPointFile
{
    int              version = support_point_format_version;
    bool             has_fingerprint = false;
    ModelFingerprint fingerprint;
    SupportPoints    points;
};

inline nlohmann::json support_points_to_json(const SupportPoints &pts,
                                             const ModelFingerprint &fp,
                                             const SupportTreeConfig &cfg)
{
    nlohmann::json j;
    j[spkey::version]           = support_point_format_version;
    j[spkey::model_fingerprint] = fingerprint_to_json(fp);

    nlohmann::json arr = nlohmann::json::array();
    for (const SupportPoint &sp : pts)
        arr.push_back(support_point_to_json(sp, cfg));
    j[spkey::points] = std::move(arr);

    return j;
}

inline bool support_points_from_json(const nlohmann::json &j,
                                     const SupportTreeConfig &cfg,
                                     SupportPointFile &out, std::string &err)
{
    err.clear(); // err is only meaningful on failure; never leave a stale one

    if (!j.is_object()) {
        err = "support point file is not a JSON object";
        return false;
    }

    // The version is checked first and on its own. An unrecognised version
    // means the keys below may not mean what they appear to mean, so nothing
    // else is read.
    const auto vit = j.find(spkey::version);
    if (vit == j.end() || !vit->is_number_integer()) {
        err = "support point file has no integer version";
        return false;
    }
    const int64_t version = vit->template get<int64_t>();
    if (version != support_point_format_version) {
        err = "unsupported support point file version " + std::to_string(version) +
              " (this build reads version " +
              std::to_string(support_point_format_version) + ")";
        return false;
    }

    SupportPointFile file;
    file.version = int(version);

    const auto fit = j.find(spkey::model_fingerprint);
    if (fit != j.end()) {
        if (!fingerprint_from_json(*fit, file.fingerprint, err))
            return false;
        file.has_fingerprint = true;
    }

    const auto pit = j.find(spkey::points);
    if (pit == j.end() || !pit->is_array()) {
        err = "support point file has no points array";
        return false;
    }

    file.points.reserve(pit->size());
    for (const nlohmann::json &pj : *pit) {
        SupportPoint sp;
        if (!support_point_from_json(pj, cfg, sp, err))
            return false;
        file.points.push_back(sp);
    }

    out = std::move(file);
    return true;
}

// ---------------------------------------------------------------------------
// String convenience. Malformed JSON becomes an error like any other, never an
// exception escaping into the CLI.
// ---------------------------------------------------------------------------

inline std::string support_points_to_string(const SupportPoints &pts,
                                            const ModelFingerprint &fp,
                                            const SupportTreeConfig &cfg,
                                            int indent = 2)
{
    return support_points_to_json(pts, fp, cfg).dump(indent);
}

inline bool support_points_from_string(const std::string &s,
                                       const SupportTreeConfig &cfg,
                                       SupportPointFile &out, std::string &err)
{
    err.clear(); // err is only meaningful on failure; never leave a stale one

    nlohmann::json j;
    try {
        j = nlohmann::json::parse(s);
    } catch (const std::exception &e) {
        err = std::string("support point file is not valid JSON: ") + e.what();
        return false;
    }
    return support_points_from_json(j, cfg, out, err);
}

}} // namespace Slic3r::sla

#endif // SLA_SUPPORTPOINTIO_HPP

///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef SLA_MODELFINGERPRINT_HPP
#define SLA_MODELFINGERPRINT_HPP

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

#include <libslic3r/Model.hpp>
#include <libslic3r/Point.hpp>
#include <libslic3r/TriangleMesh.hpp>

namespace Slic3r { namespace sla {

// ---------------------------------------------------------------------------
// Model fingerprint (openspec change per-point-support-sizing, decision D5)
//
// A support point list is only valid for the geometry it was generated from.
// The backend is a stateless one-shot process that sees nothing but the STL
// bytes, so it cannot be told "the user rotated the model" - it has to work
// that out for itself. The fingerprint is that self-check: it travels with an
// exported point list and is recomputed on import.
//
// It is deliberately NOT a hash of the uploaded file. Re-exporting an
// unmodified model can change the bytes through float formatting alone, and a
// false rejection costs the user their edits. The fingerprint describes the
// geometry instead: face count, quantized bounding box, and a checksum over
// the quantized vertex coordinates.
//
// Computed on the object's RAW mesh - no instance transform. Arrangement
// (--center) moves instances, not vertices, so it must not change the
// fingerprint.
// ---------------------------------------------------------------------------

// Quantization step for every length here: 0.1 um, in the millimetres meshes
// are stored in. Far below any print resolution, and coarse enough to absorb
// the float formatting jitter of re-exporting the same mesh.
inline constexpr double fingerprint_quantum_mm = 1e-4;

// Saturation bound for the quantized value. std::llround leaves its result
// UNSPECIFIED (and may raise FE_INVALID) once the rounded value no longer fits
// the return type, so the input is clamped before it ever gets there. 9e18 is
// exactly representable as a double and sits below INT64_MAX (~9.223e18).
inline constexpr double fingerprint_quantize_limit = 9.0e18;

// mm -> quantized integer units.
//
// The guards are applied AFTER the division, which is what makes them
// complete: that catches NaN and infinite inputs, a finite input whose scaled
// value overflows to infinity, and a finite scaled value that is simply too
// large for int64_t. Checking the raw millimetre value alone would let the
// last two through. A mesh that reaches any of these is broken; the point is
// only that the result stays deterministic instead of unspecified.
inline int64_t fingerprint_quantize(double v_mm)
{
    const double scaled = v_mm / fingerprint_quantum_mm;

    if (!std::isfinite(scaled))
        return 0;
    if (scaled >  fingerprint_quantize_limit) return  int64_t(9000000000000000000LL);
    if (scaled < -fingerprint_quantize_limit) return -int64_t(9000000000000000000LL);

    return static_cast<int64_t>(std::llround(scaled));
}

// Accumulation constants. The multiplier is the 64-bit FNV prime, used here
// purely as a large odd number that mixes well.
inline constexpr uint64_t fingerprint_seed  = 14695981039346656037ull;
inline constexpr uint64_t fingerprint_prime = 1099511628211ull;

// Quantize-then-accumulate, one quantized coordinate at a time.
//
// The multiply is what makes this ORDER DEPENDENT, and that is the point. A
// plain unordered sum cannot see a symmetric model rotated 180 degrees about
// Z: every vertex lands on the position of another vertex, so the multiset of
// coordinates - and therefore any unordered sum of it - is unchanged. Only the
// order changes. Carrying the position in the stream is what catches it.
inline uint64_t fingerprint_accumulate(uint64_t acc, int64_t q)
{
    return acc * fingerprint_prime + static_cast<uint64_t>(q);
}

// splitmix64 finalizer: spreads the last accumulated bits over the whole word
// so a one-unit change in the final vertex is not confined to the low bits.
inline uint64_t fingerprint_finalize(uint64_t acc)
{
    acc ^= acc >> 30;
    acc *= 0xbf58476d1ce4e5b9ull;
    acc ^= acc >> 27;
    acc *= 0x94d049bb133111ebull;
    acc ^= acc >> 31;
    return acc;
}

struct ModelFingerprint
{
    // Triangle count. Catches hollowing, hole drilling, remeshing.
    uint64_t face_count = 0;

    // Quantized bounding box. Catches translation, scaling, most rotations,
    // and is the part a human can read when debugging a mismatch.
    std::array<int64_t, 3> bbox_min{{0, 0, 0}};
    std::array<int64_t, 3> bbox_max{{0, 0, 0}};

    // Order-dependent checksum over the quantized vertex coordinates. Catches
    // everything the two above miss.
    uint64_t vertex_checksum = 0;

    bool operator==(const ModelFingerprint &o) const
    {
        return face_count == o.face_count && bbox_min == o.bbox_min &&
               bbox_max == o.bbox_max && vertex_checksum == o.vertex_checksum;
    }

    bool operator!=(const ModelFingerprint &o) const { return !(*this == o); }
};

// The fingerprint of a raw mesh. No transform of any kind is applied here -
// the caller is responsible for handing in the untransformed geometry.
inline ModelFingerprint model_fingerprint(const indexed_triangle_set &its)
{
    ModelFingerprint fp;
    fp.face_count = static_cast<uint64_t>(its.indices.size());

    uint64_t acc = fingerprint_seed;

    for (size_t i = 0; i < its.vertices.size(); ++i) {
        const Vec3f &v = its.vertices[i];

        std::array<int64_t, 3> q{{fingerprint_quantize(double(v.x())),
                                  fingerprint_quantize(double(v.y())),
                                  fingerprint_quantize(double(v.z()))}};

        if (i == 0) {
            fp.bbox_min = q;
            fp.bbox_max = q;
        } else {
            for (int k = 0; k < 3; ++k) {
                fp.bbox_min[k] = std::min(fp.bbox_min[k], q[k]);
                fp.bbox_max[k] = std::max(fp.bbox_max[k], q[k]);
            }
        }

        for (int k = 0; k < 3; ++k)
            acc = fingerprint_accumulate(acc, q[k]);
    }

    fp.vertex_checksum = fingerprint_finalize(acc);
    return fp;
}

// The fingerprint of a ModelObject.
//
// raw_indexed_triangle_set() is the sum of the non-modifier volumes with their
// volume matrices applied and NO instance transform - which is exactly the
// contract here. Arrangement, --center and instance rotation all live on the
// instance, so none of them reach this.
inline ModelFingerprint model_fingerprint(const ModelObject &mo)
{
    return model_fingerprint(mo.raw_indexed_triangle_set());
}

// Two fingerprints describe the same geometry. Spelled out as a named function
// so import sites read as intent rather than as an operator on a struct.
inline bool fingerprint_matches(const ModelFingerprint &a,
                                const ModelFingerprint &b)
{
    return a == b;
}

// ---------------------------------------------------------------------------
// Serialization
//
// The checksum travels as a fixed-width hex string rather than a number: it
// uses the full 64-bit range, and JSON consumers that parse numbers as IEEE
// doubles would silently round the top bits away.
// ---------------------------------------------------------------------------

inline std::string fingerprint_checksum_to_string(uint64_t v)
{
    std::ostringstream ss;
    ss << std::hex << std::setw(16) << std::setfill('0') << v;
    return ss.str();
}

// The writer always emits exactly 16 zero-padded hex digits, so the reader
// demands exactly 16. Accepting a shorter run would turn a truncated file into
// a well-formed but wrong fingerprint, which the user would then be told is a
// changed model rather than a damaged file.
inline bool fingerprint_checksum_from_string(const std::string &s, uint64_t &out)
{
    if (s.size() != 16)
        return false;
    uint64_t v = 0;
    for (char c : s) {
        int d;
        if (c >= '0' && c <= '9')      d = c - '0';
        else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
        else return false;
        v = (v << 4) | static_cast<uint64_t>(d);
    }
    out = v;
    return true;
}

// One canonical line, for logs and for round-trip tests. The JSON form of the
// same data is defined by the interchange format, not here.
inline std::string fingerprint_to_string(const ModelFingerprint &fp)
{
    std::ostringstream ss;
    ss << "faces=" << fp.face_count << ";bbox=";
    for (int k = 0; k < 3; ++k) ss << fp.bbox_min[k] << ',';
    for (int k = 0; k < 3; ++k) ss << fp.bbox_max[k] << ',';
    ss << ";checksum=" << fingerprint_checksum_to_string(fp.vertex_checksum);
    return ss.str();
}

inline bool fingerprint_from_string(const std::string &s, ModelFingerprint &out)
{
    ModelFingerprint fp;
    std::istringstream ss(s);
    std::string tok;

    auto expect = [&ss](const char *lit) {
        for (const char *p = lit; *p; ++p)
            if (ss.get() != *p) return false;
        return true;
    };

    if (!expect("faces=")) return false;
    {
        // Read through a signed type on purpose: operator>> for an unsigned
        // type accepts a leading '-' and wraps it, so "faces=-1" would parse
        // "successfully" as 18446744073709551615.
        int64_t faces = 0;
        if (!(ss >> faces) || faces < 0) return false;
        fp.face_count = static_cast<uint64_t>(faces);
    }
    if (!expect(";bbox=")) return false;
    for (int k = 0; k < 3; ++k) {
        if (!(ss >> fp.bbox_min[k])) return false;
        if (ss.get() != ',') return false;
    }
    for (int k = 0; k < 3; ++k) {
        if (!(ss >> fp.bbox_max[k])) return false;
        if (ss.get() != ',') return false;
    }
    if (!expect(";checksum=")) return false;
    if (!(ss >> tok)) return false;
    if (!fingerprint_checksum_from_string(tok, fp.vertex_checksum)) return false;

    // Nothing but whitespace may follow. Without this a half-written or
    // concatenated line parses as valid.
    for (int c = ss.get(); c != std::char_traits<char>::eof(); c = ss.get())
        if (!std::isspace(static_cast<unsigned char>(c)))
            return false;

    out = fp;
    return true;
}

}} // namespace Slic3r::sla

#endif // SLA_MODELFINGERPRINT_HPP

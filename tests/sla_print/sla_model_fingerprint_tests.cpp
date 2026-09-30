#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/SLA/ModelFingerprint.hpp"

using namespace Slic3r;

// Model fingerprint coverage (openspec change per-point-support-sizing,
// tasks 4.4 - 4.6). Two properties are being pinned here and they pull in
// opposite directions:
//
//   * it must NOT change for things that leave the geometry alone - a
//     different arrangement, a re-export, a last-bit float difference. A false
//     mismatch throws away the user's support point edits.
//   * it MUST change for anything that moves a vertex, including the awkward
//     case of a symmetric model turned 180 degrees, where the face count and
//     the bounding box both stay put.

namespace {

indexed_triangle_set transformed(const indexed_triangle_set &in,
                                 const Transform3d &t)
{
    indexed_triangle_set out = in;
    for (Vec3f &v : out.vertices)
        v = (t * v.cast<double>()).cast<float>();
    return out;
}

// A cylinder is symmetric about Z: with an even facet count, turning it 180
// degrees maps every vertex onto the position of another one. Face count and
// bounding box are unchanged, only the order of the vertices differs - which
// is the case only an order-dependent checksum can catch.
indexed_triangle_set symmetric_mesh()
{
    // The default facet angle gives 360 steps, so the 180 degree turn maps
    // step i onto step i + 180 exactly.
    return its_make_cylinder(4., 10.);
}

// A prism whose cross section is closed under (x, y) -> (-x, -y). Every
// coordinate is exactly representable as a float AND an exact multiple of the
// quantization step, so a half turn about Z is a plain negation: no rounding
// enters, nothing lands on a bucket boundary, and the multiset of quantized
// coordinates is preserved bit for bit. That is what makes this the mesh to
// state the premise on.
indexed_triangle_set exactly_symmetric_prism()
{
    const float ring[6][2] = {{ 5.0f,  0.0f}, { 2.5f,  4.5f}, {-2.5f,  4.5f},
                              {-5.0f,  0.0f}, {-2.5f, -4.5f}, { 2.5f, -4.5f}};
    indexed_triangle_set its;
    for (int i = 0; i < 6; ++i) {
        its.vertices.emplace_back(Vec3f(ring[i][0], ring[i][1], 0.f));
        its.vertices.emplace_back(Vec3f(ring[i][0], ring[i][1], 8.f));
    }
    for (int i = 0; i < 6; ++i) {
        const int a = 2 * i, b = 2 * ((i + 1) % 6);
        its.indices.emplace_back(a, b, a + 1);
        its.indices.emplace_back(b, b + 1, a + 1);
    }
    return its;
}

// A 180 degree turn about Z written as the negation it is, so that no floating
// point error is introduced by going through a rotation matrix.
indexed_triangle_set turned_half_turn(const indexed_triangle_set &in)
{
    indexed_triangle_set out = in;
    for (Vec3f &v : out.vertices) {
        v.x() = -v.x();
        v.y() = -v.y();
    }
    return out;
}

// The exact stream of integers the checksum consumes, in mesh order.
std::vector<int64_t> quantized_coords(const indexed_triangle_set &its)
{
    std::vector<int64_t> out;
    out.reserve(its.vertices.size() * 3);
    for (const Vec3f &v : its.vertices) {
        out.push_back(sla::fingerprint_quantize(double(v.x())));
        out.push_back(sla::fingerprint_quantize(double(v.y())));
        out.push_back(sla::fingerprint_quantize(double(v.z())));
    }
    return out;
}

} // namespace

TEST_CASE("Arrangement does not change the fingerprint", "[sla_model_fingerprint]")
{
    Model model;
    ModelObject *mo = model.add_object();
    mo->add_volume(TriangleMesh{its_make_cube(10., 10., 10.)});
    mo->add_instance();

    mo->instances.front()->set_offset(Vec3d(0., 0., 0.));
    const sla::ModelFingerprint a = sla::model_fingerprint(*mo);

    // What --center does: it moves the instance, not the mesh.
    mo->instances.front()->set_offset(Vec3d(97.82, 61.1, 0.));
    const sla::ModelFingerprint b = sla::model_fingerprint(*mo);

    // ... and rotating the instance must not reach the raw mesh either.
    mo->instances.front()->set_rotation(Vec3d(0., 0., 0.7));
    const sla::ModelFingerprint c = sla::model_fingerprint(*mo);

    REQUIRE(a == b);
    REQUIRE(a == c);
    REQUIRE(sla::fingerprint_matches(a, b));
}

TEST_CASE("The same mesh fingerprints identically every time",
          "[sla_model_fingerprint]")
{
    const indexed_triangle_set m = its_make_cube(10., 10., 10.);

    const sla::ModelFingerprint a = sla::model_fingerprint(m);
    const sla::ModelFingerprint b = sla::model_fingerprint(m);
    // A copy is a distinct object with the same geometry.
    const indexed_triangle_set copy = m;
    const sla::ModelFingerprint c = sla::model_fingerprint(copy);

    REQUIRE(a == b);
    REQUIRE(a == c);
}

TEST_CASE("A last-bit float difference does not change the fingerprint",
          "[sla_model_fingerprint]")
{
    // One ULP near 10 mm is about 1e-6 mm, roughly a five-hundredth of the
    // 1e-4 mm quantum, so it must be absorbed. (Quantization is still
    // quantization: a value sitting exactly on a bucket edge could tip over.
    // The cube's coordinates are 0 and 10, both far from an edge.)
    indexed_triangle_set a = its_make_cube(10., 10., 10.);
    indexed_triangle_set b = a;

    bool nudged = false;
    for (Vec3f &v : b.vertices)
        for (int k = 0; k < 3; ++k)
            if (v[k] != 0.f && !nudged) {
                v[k] = std::nextafter(v[k], 1e9f);
                nudged = true;
            }
    REQUIRE(nudged);
    REQUIRE(a.vertices != b.vertices); // the meshes really do differ in bits

    REQUIRE(sla::model_fingerprint(a) == sla::model_fingerprint(b));
}

TEST_CASE("Vertex translation changes the fingerprint", "[sla_model_fingerprint]")
{
    const indexed_triangle_set base = its_make_cube(10., 10., 10.);
    const indexed_triangle_set moved =
        transformed(base, Transform3d{Eigen::Translation3d(5., 0., 0.)});

    REQUIRE(sla::model_fingerprint(base) != sla::model_fingerprint(moved));
}

TEST_CASE("Rotation changes the fingerprint", "[sla_model_fingerprint]")
{
    const indexed_triangle_set base = its_make_cube(10., 10., 10.);
    Transform3d rot = Transform3d::Identity();
    rot.rotate(Eigen::AngleAxisd(15. * M_PI / 180., Vec3d::UnitY()));

    REQUIRE(sla::model_fingerprint(base) !=
            sla::model_fingerprint(transformed(base, rot)));
}

TEST_CASE("Scaling changes the fingerprint", "[sla_model_fingerprint]")
{
    const indexed_triangle_set base = its_make_cube(10., 10., 10.);
    Transform3d scl = Transform3d::Identity();
    scl.scale(1.1);

    REQUIRE(sla::model_fingerprint(base) !=
            sla::model_fingerprint(transformed(base, scl)));
}

TEST_CASE("A face count change is detected", "[sla_model_fingerprint]")
{
    // Any operation that EDITS THE MESH lands here as a different triangle
    // count on the same object.
    //
    // Do not read this as cover for hollowing or hole drilling in this fork:
    // both are pipeline settings (slaposHollowing / slaposDrillHoles) that
    // leave ModelObject::mesh() alone, so they do NOT move the fingerprint.
    // The next test pins that deliberately. Only a mesh whose own triangles
    // changed is caught here.
    const indexed_triangle_set base = its_make_cube(10., 10., 10.);
    indexed_triangle_set fewer = base;
    REQUIRE(fewer.indices.size() > 1);
    fewer.indices.pop_back();

    const sla::ModelFingerprint a = sla::model_fingerprint(base);
    const sla::ModelFingerprint b = sla::model_fingerprint(fewer);

    REQUIRE(a.face_count != b.face_count);
    REQUIRE(a != b);
}

TEST_CASE("The same mesh keeps its fingerprint whatever the print settings say",
          "[sla_model_fingerprint]")
{
    // The boundary of what the fingerprint promises.
    //
    // model_fingerprint() takes an indexed_triangle_set and nothing else: no
    // config, no pipeline state. So a setting that changes what gets BUILT
    // without changing the mesh - hollowing_enable, drain holes - cannot move
    // the value, and an already exported point list stays importable across
    // such a change. That is deliberate: hollowing carves the inside only, the
    // outer surface the points sit on does not move.
    //
    // Pinned as a test because the opposite reading is the intuitive one, and
    // acting on it (expecting a hollow to invalidate a point list) would send
    // someone hunting for a bug that is not there.
    const indexed_triangle_set mesh = its_make_cube(10., 10., 10.);

    REQUIRE(sla::model_fingerprint(mesh) == sla::model_fingerprint(mesh));

    // A copy that no one edited fingerprints the same, however it got here.
    const indexed_triangle_set untouched_copy = mesh;
    REQUIRE(sla::model_fingerprint(mesh) == sla::model_fingerprint(untouched_copy));
}

TEST_CASE("A single vertex moved by 1 um is detected", "[sla_model_fingerprint]")
{
    // 1 um is ten quantization steps, comfortably above the threshold.
    const indexed_triangle_set base = its_make_cube(10., 10., 10.);
    indexed_triangle_set nudged = base;
    nudged.vertices.front().x() += 0.001f;

    REQUIRE(sla::model_fingerprint(base) != sla::model_fingerprint(nudged));
}

TEST_CASE("A symmetric model turned 180 degrees is caught by the checksum",
          "[sla_model_fingerprint]")
{
    const indexed_triangle_set base = symmetric_mesh();

    Transform3d rot = Transform3d::Identity();
    rot.rotate(Eigen::AngleAxisd(M_PI, Vec3d::UnitZ()));
    const indexed_triangle_set turned = transformed(base, rot);

    // Note on this mesh: the turn very nearly permutes the quantized
    // coordinates, but not exactly. Two of the 1440 of them sit on a
    // quantization boundary and shift by one unit. That is harmless here, but
    // it does mean this mesh cannot be used to pin down the premise behind the
    // order-dependent checksum - the test below does that instead.
    REQUIRE(quantized_coords(base) != quantized_coords(turned));

    const sla::ModelFingerprint a = sla::model_fingerprint(base);
    const sla::ModelFingerprint b = sla::model_fingerprint(turned);

    // The two coarse components genuinely cannot see this...
    REQUIRE(a.face_count == b.face_count);
    REQUIRE(a.bbox_min == b.bbox_min);
    REQUIRE(a.bbox_max == b.bbox_max);
    // ... so the checksum is the only thing standing between the user and a
    // set of support points rotated half a turn off the model.
    REQUIRE(a.vertex_checksum != b.vertex_checksum);
    REQUIRE(a != b);
}

TEST_CASE("Only the ORDER of the coordinates changes under a half turn",
          "[sla_model_fingerprint]")
{
    // This is the test that justifies the checksum being order dependent, and
    // the one that must fail if anybody ever simplifies it back to an
    // unordered form.
    const indexed_triangle_set base   = exactly_symmetric_prism();
    const indexed_triangle_set turned = turned_half_turn(base);

    std::vector<int64_t> qa = quantized_coords(base);
    std::vector<int64_t> qb = quantized_coords(turned);

    // The stream the checksum walks really is a different stream ...
    REQUIRE(qa != qb);

    std::sort(qa.begin(), qa.end());
    std::sort(qb.begin(), qb.end());
    // ... yet it holds exactly the same numbers. Any unordered accumulation -
    // a plain sum included - is blind to this by construction.
    REQUIRE(qa == qb);

    const sla::ModelFingerprint a = sla::model_fingerprint(base);
    const sla::ModelFingerprint b = sla::model_fingerprint(turned);

    // Every coarse component is identical, exactly as in the multiset above.
    REQUIRE(a.face_count == b.face_count);
    REQUIRE(a.bbox_min == b.bbox_min);
    REQUIRE(a.bbox_max == b.bbox_max);

    // So the order-dependent checksum is the only thing that can tell them
    // apart - and it does.
    REQUIRE(a.vertex_checksum != b.vertex_checksum);
    REQUIRE(a != b);
}

TEST_CASE("Fingerprints survive a string round trip", "[sla_model_fingerprint]")
{
    const sla::ModelFingerprint fp =
        sla::model_fingerprint(its_make_cube(10., 20., 30.));

    sla::ModelFingerprint back;
    REQUIRE(sla::fingerprint_from_string(sla::fingerprint_to_string(fp), back));
    REQUIRE(back == fp);

    // The checksum keeps all 64 bits, including a leading-zero one.
    for (uint64_t v : {uint64_t(0), uint64_t(1), ~uint64_t(0),
                       uint64_t(0x00ff00ff00ff00ffull)}) {
        uint64_t out = 12345;
        REQUIRE(sla::fingerprint_checksum_from_string(
            sla::fingerprint_checksum_to_string(v), out));
        REQUIRE(out == v);
    }

    // Garbage is rejected rather than silently parsed as zero.
    sla::ModelFingerprint ignored;
    REQUIRE_FALSE(sla::fingerprint_from_string("", ignored));
    REQUIRE_FALSE(sla::fingerprint_from_string("faces=abc;", ignored));
    uint64_t cs = 0;
    REQUIRE_FALSE(sla::fingerprint_checksum_from_string("xyz", cs));
    REQUIRE_FALSE(sla::fingerprint_checksum_from_string("", cs));
}

TEST_CASE("Quantization stays in range for absurd coordinates",
          "[sla_model_fingerprint]")
{
    // Nothing here may reach std::llround with an out-of-range argument, where
    // the result would be unspecified. Only a broken mesh gets this far; the
    // requirement is just that the answer is deterministic.
    REQUIRE(sla::fingerprint_quantize(0.) == 0);
    REQUIRE(sla::fingerprint_quantize(-0.) == 0);
    REQUIRE(sla::fingerprint_quantize(1.) == 10000);
    REQUIRE(sla::fingerprint_quantize(-1.) == -10000);

    // Non-finite inputs.
    REQUIRE(sla::fingerprint_quantize(std::numeric_limits<double>::quiet_NaN()) == 0);
    REQUIRE(sla::fingerprint_quantize(std::numeric_limits<double>::infinity()) == 0);
    REQUIRE(sla::fingerprint_quantize(-std::numeric_limits<double>::infinity()) == 0);

    // Finite, but the division alone overflows to infinity.
    REQUIRE(sla::fingerprint_quantize(std::numeric_limits<double>::max()) == 0);

    // Finite and finite after scaling, but far past what int64_t holds. These
    // saturate rather than wandering off.
    const int64_t hi = sla::fingerprint_quantize(1e30);
    const int64_t lo = sla::fingerprint_quantize(-1e30);
    REQUIRE(hi > 0);
    REQUIRE(lo < 0);
    REQUIRE(hi == -lo);
    REQUIRE(hi == sla::fingerprint_quantize(1e40)); // saturated, so equal

    // A mesh full of them still fingerprints without tripping anything.
    indexed_triangle_set broken = its_make_cube(10., 10., 10.);
    broken.vertices.front().x() = std::numeric_limits<float>::infinity();
    REQUIRE(sla::model_fingerprint(broken) == sla::model_fingerprint(broken));
}

TEST_CASE("Malformed fingerprint strings are rejected", "[sla_model_fingerprint]")
{
    const sla::ModelFingerprint fp =
        sla::model_fingerprint(its_make_cube(10., 20., 30.));
    const std::string good = sla::fingerprint_to_string(fp);

    sla::ModelFingerprint out;
    REQUIRE(sla::fingerprint_from_string(good, out));

    // Trailing garbage. A concatenated or half-overwritten line must not read
    // as a valid fingerprint.
    REQUIRE_FALSE(sla::fingerprint_from_string(good + " junk", out));
    REQUIRE_FALSE(sla::fingerprint_from_string(good + "0", out));
    // Trailing whitespace is still fine - files end with a newline.
    REQUIRE(sla::fingerprint_from_string(good + "\n", out));
    REQUIRE(sla::fingerprint_from_string(good + "  \t\n", out));

    // A negative face count must not wrap into a huge unsigned one.
    REQUIRE_FALSE(sla::fingerprint_from_string(
        "faces=-1;bbox=0,0,0,1,1,1,;checksum=0000000000000000", out));

    // The checksum is exactly 16 hex digits, no more and no less.
    REQUIRE_FALSE(sla::fingerprint_checksum_from_string("0", out.vertex_checksum));
    REQUIRE_FALSE(sla::fingerprint_checksum_from_string("000000000000000", // 15
                                                        out.vertex_checksum));
    REQUIRE_FALSE(sla::fingerprint_checksum_from_string("00000000000000000", // 17
                                                        out.vertex_checksum));
    REQUIRE(sla::fingerprint_checksum_from_string("000000000000000f",
                                                  out.vertex_checksum));
    REQUIRE(out.vertex_checksum == 15u);
    // Upper case stays acceptable.
    REQUIRE(sla::fingerprint_checksum_from_string("00000000000000FF",
                                                  out.vertex_checksum));
    REQUIRE(out.vertex_checksum == 255u);
}

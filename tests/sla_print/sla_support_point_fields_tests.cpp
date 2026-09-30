#include <catch2/catch_test_macros.hpp>

#include <sstream>

#include <cereal/archives/binary.hpp>

#include "libslic3r/SLA/SupportPoint.hpp"

using namespace Slic3r;
using namespace Slic3r::sla;

// Unit coverage for the per-point geometry fields added by the openspec change
// per-point-support-sizing (tasks 2.1 - 2.6). Three things are locked here,
// each guarding a failure mode that is silent rather than loud:
//
//   * cereal round trip  - a field missing from serialize() is dropped without
//     any error on every undo/redo and backend snapshot.
//   * operator==         - a field missing from the comparison makes change
//     detection reuse a stale support tree.
//   * contact_sphere_radius tri-state - folding "== 0" into a "<= 0 means
//     unset" test turns "this point has no contact sphere" into "use the
//     global default", which is the opposite instruction.

namespace {

// A point with head_front_radius plus all six extension fields set to
// distinct, non-sentinel values, so that a swap or a copy-paste slip between
// two fields is visible.
SupportPoint make_fully_specified_point()
{
    SupportPoint sp;
    sp.pos                       = Vec3f(1.f, 2.f, 3.f);
    sp.type                      = SupportPointType::slope;
    sp.head_front_radius         = 0.21f;
    sp.head_back_radius_mm       = 0.52f;
    sp.head_width_mm             = 1.03f;
    sp.head_penetration_mm       = 0.34f;
    sp.contact_sphere_radius     = 0.45f;
    sp.base_radius_mm            = 2.06f;
    sp.support_bracing_angle_deg = 37.f;
    return sp;
}

// The target is pre-filled with values that match neither the source nor the
// struct defaults. Without that, a field dropped from serialize() would leave
// the target at its default and the assertions would still pass whenever the
// expected value happens to be that default - the round trip would be checking
// nothing.
SupportPoint poisoned_target()
{
    SupportPoint sp;
    sp.pos                       = Vec3f(-99.f, -98.f, -97.f);
    sp.type                      = SupportPointType::island;
    sp.head_front_radius         = 9.1f;
    sp.head_back_radius_mm       = 9.2f;
    sp.head_width_mm             = 9.3f;
    sp.head_penetration_mm       = 9.4f;
    sp.contact_sphere_radius     = 9.5f;
    sp.base_radius_mm            = 9.6f;
    sp.support_bracing_angle_deg = 9.7f;
    return sp;
}

SupportPoint cereal_round_trip(const SupportPoint &in)
{
    std::stringstream ss;
    {
        cereal::BinaryOutputArchive oar(ss);
        // serialize() is non-const, so archive a copy.
        SupportPoint tmp = in;
        oar(tmp);
    }
    SupportPoint out = poisoned_target();
    {
        cereal::BinaryInputArchive iar(ss);
        iar(out);
    }
    return out;
}

} // namespace

TEST_CASE("SupportPoint size fields default to the unset sentinel", "[sla_support_point_fields]")
{
    SupportPoint sp;

    REQUIRE(support_point_unset == -1.f);
    REQUIRE(sp.head_back_radius_mm == support_point_unset);
    REQUIRE(sp.head_width_mm == support_point_unset);
    REQUIRE(sp.head_penetration_mm == support_point_unset);
    REQUIRE(sp.contact_sphere_radius == support_point_unset);
    REQUIRE(sp.base_radius_mm == support_point_unset);
    REQUIRE(sp.support_bracing_angle_deg == support_point_unset);

    // head_front_radius keeps its historical 0 default and is deliberately
    // outside the sentinel scheme: -1 there means "marked for erase" in
    // prepare_permanent_support_points(), so it has no point_*() resolver and
    // must never be filled with the sentinel.
    REQUIRE(sp.head_front_radius == 0.f);
    REQUIRE(sp.head_front_radius != support_point_unset);
}

TEST_CASE("SupportPoint survives a cereal round trip field by field", "[sla_support_point_fields]")
{
    const SupportPoint in  = make_fully_specified_point();
    const SupportPoint out = cereal_round_trip(in);

    REQUIRE(out.pos == in.pos);
    REQUIRE(out.type == in.type);
    REQUIRE(out.head_front_radius == in.head_front_radius);
    REQUIRE(out.head_back_radius_mm == in.head_back_radius_mm);
    REQUIRE(out.head_width_mm == in.head_width_mm);
    REQUIRE(out.head_penetration_mm == in.head_penetration_mm);
    REQUIRE(out.contact_sphere_radius == in.contact_sphere_radius);
    REQUIRE(out.base_radius_mm == in.base_radius_mm);
    REQUIRE(out.support_bracing_angle_deg == in.support_bracing_angle_deg);
    REQUIRE(out == in);
}

TEST_CASE("Sentinel values also survive a cereal round trip", "[sla_support_point_fields]")
{
    SupportPoint in;
    in.pos = Vec3f(4.f, 5.f, 6.f);

    const SupportPoint out = cereal_round_trip(in);

    // The target started poisoned (see poisoned_target()), so every one of
    // these can only hold the sentinel if the field really came across.
    REQUIRE(out.head_back_radius_mm == support_point_unset);
    REQUIRE(out.head_width_mm == support_point_unset);
    REQUIRE(out.head_penetration_mm == support_point_unset);
    REQUIRE(out.contact_sphere_radius == support_point_unset);
    REQUIRE(out.base_radius_mm == support_point_unset);
    REQUIRE(out.support_bracing_angle_deg == support_point_unset);
    REQUIRE(out.head_front_radius == 0.f);
    REQUIRE(out.pos == in.pos);
    REQUIRE(out.type == in.type);
    REQUIRE(out == in);
}

TEST_CASE("A difference in any single size field is detected", "[sla_support_point_fields]")
{
    const SupportPoint base = make_fully_specified_point();

    auto differs_in = [&base](float SupportPoint::*field, float value) {
        SupportPoint other = base;
        other.*field = value;
        REQUIRE(other != base);
        REQUIRE_FALSE(other == base);
    };

    differs_in(&SupportPoint::head_front_radius, 0.31f);
    differs_in(&SupportPoint::head_back_radius_mm, 0.62f);
    differs_in(&SupportPoint::head_width_mm, 1.53f);
    differs_in(&SupportPoint::head_penetration_mm, 0.44f);
    differs_in(&SupportPoint::contact_sphere_radius, 0.65f);
    differs_in(&SupportPoint::base_radius_mm, 3.06f);
    differs_in(&SupportPoint::support_bracing_angle_deg, 42.f);

    // Setting a field back to the sentinel is an edit too, not a "no value to
    // compare" free pass.
    differs_in(&SupportPoint::contact_sphere_radius, support_point_unset);
    differs_in(&SupportPoint::base_radius_mm, support_point_unset);

    // An identical copy still compares equal.
    SupportPoint same = base;
    REQUIRE(same == base);
    REQUIRE_FALSE(same != base);
}

TEST_CASE("Size fields resolve to the point value when set, global otherwise",
          "[sla_support_point_fields]")
{
    SupportPoint sp;

    SECTION("all sentinels fall back to the global defaults") {
        REQUIRE(point_head_back_radius_mm(sp, 0.5) == 0.5);
        REQUIRE(point_head_width_mm(sp, 1.0) == 1.0);
        REQUIRE(point_head_penetration_mm(sp, 0.3) == 0.3);
        REQUIRE(point_base_radius_mm(sp, 2.0) == 2.0);
    }

    SECTION("a set field wins over the global default") {
        sp.head_back_radius_mm = 0.6f;
        // double(0.6f) != 0.6: the field is a float, so compare against the
        // widened float rather than the double literal.
        REQUIRE(point_head_back_radius_mm(sp, 0.5) == double(0.6f));
        // The neighbouring fields are untouched.
        REQUIRE(point_head_width_mm(sp, 1.0) == 1.0);
    }

    SECTION("zero is a set value for the plain size fields") {
        sp.head_penetration_mm = 0.f;
        REQUIRE(point_field_is_set(sp.head_penetration_mm));
        REQUIRE(point_head_penetration_mm(sp, 0.3) == 0.);
    }

    SECTION("the bracing angle converts degrees to radians") {
        sp.support_bracing_angle_deg = 30.f;
        const double expected = 30. * support_point_deg_to_rad;
        REQUIRE(point_bracing_angle_rad(sp, 1.234) == expected);

        sp.support_bracing_angle_deg = support_point_unset;
        REQUIRE(point_bracing_angle_rad(sp, 1.234) == 1.234);
    }
}

TEST_CASE("contact_sphere_radius resolves as three distinct states",
          "[sla_support_point_fields]")
{
    SupportPoint sp;

    SECTION("negative means not set: the global setting applies") {
        sp.contact_sphere_radius = support_point_unset;
        REQUIRE(point_contact_sphere_state(sp) == ContactSphereState::UseGlobalDefault);

        const ContactSphere cs = point_contact_sphere(sp, true, 0.4);
        REQUIRE(cs.enabled);
        REQUIRE(cs.radius_mm == 0.4);

        // ... including when the global setting is off.
        const ContactSphere off = point_contact_sphere(sp, false, 0.4);
        REQUIRE_FALSE(off.enabled);
    }

    SECTION("zero means explicitly off, even with the global setting on") {
        sp.contact_sphere_radius = 0.f;
        REQUIRE(point_contact_sphere_state(sp) == ContactSphereState::Disabled);

        const ContactSphere cs = point_contact_sphere(sp, true, 0.4);
        REQUIRE_FALSE(cs.enabled);
        REQUIRE(cs.radius_mm == 0.);
    }

    SECTION("positive is the radius, regardless of the global setting") {
        sp.contact_sphere_radius = 0.7f;
        REQUIRE(point_contact_sphere_state(sp) == ContactSphereState::Custom);

        const ContactSphere on = point_contact_sphere(sp, true, 0.4);
        REQUIRE(on.enabled);
        REQUIRE(on.radius_mm == 0.7f);

        const ContactSphere off = point_contact_sphere(sp, false, 0.4);
        REQUIRE(off.enabled);
        REQUIRE(off.radius_mm == 0.7f);
    }

    SECTION("zero and the sentinel are never the same state") {
        SupportPoint zero;
        zero.contact_sphere_radius = 0.f;
        SupportPoint unset;
        unset.contact_sphere_radius = support_point_unset;

        REQUIRE(point_contact_sphere_state(zero) != point_contact_sphere_state(unset));
        REQUIRE(point_contact_sphere(zero, true, 0.4).enabled !=
                point_contact_sphere(unset, true, 0.4).enabled);
        REQUIRE(zero != unset);
    }
}

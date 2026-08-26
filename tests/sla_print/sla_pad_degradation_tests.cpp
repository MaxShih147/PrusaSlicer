#include <catch2/catch_test_macros.hpp>
#include <test_utils.hpp>

#include "libslic3r/Exception.hpp"
#include "libslic3r/FileReader.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/SLAPrint.hpp"
#include "libslic3r/SLA/Pad.hpp"
#include "libslic3r/TriangleMesh.hpp"

using namespace Slic3r;

// Guards the pad-step degradation added by change fix-empty-pad-slicing-error
// (capability sla-pad-generation).
//
// Outside zero-elevation mode a pad can only be grown from the support tree's
// footprint. When supports are requested but no pillar comes out, the pad is
// necessarily empty — which used to throw SlicingError, aborting process()
// before the support STL export ran and turning a "nothing to build" state into
// a hard error. generate_pad() now degrades in exactly that case and keeps
// throwing everywhere else.
//
// Fixture note: 20mm_cube.obj is NOT a zero-support model. Its bottom face has
// normal (0,0,-1) => polar == PI, which survives the overhang filter even at the
// maximum threshold, so pinheads are placed. Rotating it about Y removes every
// exactly-horizontal down-facing face, and at support_critical_angle = 90 the
// tree comes out empty — the same mechanism as the reported 3DBenchy case.
// Both behaviours were confirmed against the CLI before this test was written.

namespace {

constexpr double MAX_OVERHANG_THRESHOLD_DEG = 90.;
constexpr double ZERO_SUPPORT_ROTATION_DEG  = 40.;

// An SLA print of the 20mm test cube, optionally rotated about Y, stopped right
// after the pad step (the same set_task() trick the CLI uses for
// --export-support-stl). Stopping there keeps the run focused on generate_pad
// and skips slice-supports + rasterization.
struct CubePadPrint
{
    Model    model;
    SLAPrint print;

    CubePadPrint(bool supports_enable, bool pad_enable, double rot_y_deg)
        : model(FileReader::load_model(TEST_DATA_DIR PATH_SEPARATOR
                                       + std::string("20mm_cube") + ".obj"))
    {
        if (rot_y_deg != 0.)
            // Mirrors the CLI's --rotate-y: rotates the volumes, then recenters.
            for (ModelObject *o : model.objects)
                o->rotate(Geometry::deg2rad(rot_y_deg), Y);

        SLAFullPrintConfig fullcfg;
        fullcfg.printer_technology.setInt(ptSLA);
        fullcfg.supports_enable.value = supports_enable;
        fullcfg.pad_enable.value      = pad_enable;
        // Pinned explicitly rather than relying on the default: capability
        // sla-overhang-threshold-semantics freezes "larger value => fewer
        // supports", and this whole fixture depends on that direction.
        fullcfg.support_critical_angle.value = MAX_OVERHANG_THRESHOLD_DEG;

        DynamicPrintConfig cfg;
        cfg.apply(fullcfg);

        print.set_status_callback([](const PrintBase::SlicingStatus &) {});
        print.apply(model, cfg);

        PrintBase::TaskParams task;
        task.to_object_step = slaposPad;
        print.set_task(task);
    }

    const SLAPrintObject& object() const { return *print.objects().front(); }
};

} // namespace

// ── Degradation: the case the change exists for ───────────────────────────────

TEST_CASE("Empty support tree with pad enabled degrades instead of throwing",
          "[sla_pad_degradation]")
{
    CubePadPrint run{/*supports*/ true, /*pad*/ true, ZERO_SUPPORT_ROTATION_DEG};

    REQUIRE_NOTHROW(run.print.process());

    const SLAPrintObject &po = run.object();

    // The step must complete, not be left un-run: the CLI's support export keys
    // off is_step_done(slaposPad) to decide what to report.
    REQUIRE(po.is_step_done(slaposSupportTree));
    REQUIRE(po.is_step_done(slaposPad));

    // Precondition of the degradation: no pillar was generated at this angle.
    REQUIRE(po.support_mesh().empty());

    // And the outcome: an empty pad, same end state as the pad-disabled branch.
    REQUIRE(po.pad_mesh().empty());
}

TEST_CASE("Zero-support model with pad disabled is unchanged",
          "[sla_pad_degradation]")
{
    // The pad-disabled branch is the end state the degradation converges to.
    // Kept as a companion so a regression on either side shows up as a
    // divergence between the two.
    CubePadPrint run{/*supports*/ true, /*pad*/ false, ZERO_SUPPORT_ROTATION_DEG};

    REQUIRE_NOTHROW(run.print.process());

    const SLAPrintObject &po = run.object();
    REQUIRE(po.is_step_done(slaposPad));
    REQUIRE(po.support_mesh().empty());
}

// ── Fail-closed: everything else must still abort ─────────────────────────────

TEST_CASE("Pad enabled without supports still fails closed",
          "[sla_pad_degradation]")
{
    // supports_enable = 0 leaves the tree empty too, so the degradation would
    // swallow this case if the guard were only "empty pad && empty tree". The
    // supports_enable term keeps it out, and an invalid pad here still aborts.
    // (Verified against the CLI: this configuration fails on this fork for both
    // the flat and the rotated cube — pre-existing behaviour, deliberately
    // preserved.)
    CubePadPrint run{/*supports*/ false, /*pad*/ true, ZERO_SUPPORT_ROTATION_DEG};

    REQUIRE_THROWS_AS(run.print.process(), Slic3r::SlicingError);
}

TEST_CASE("Pad enabled without supports fails closed for a flat model too",
          "[sla_pad_degradation]")
{
    CubePadPrint run{/*supports*/ false, /*pad*/ true, 0.};

    REQUIRE_THROWS_AS(run.print.process(), Slic3r::SlicingError);
}

// ── The normal path must be untouched ─────────────────────────────────────────

TEST_CASE("Model that needs supports still gets supports and a pad",
          "[sla_pad_degradation]")
{
    // Unrotated: the flat bottom face survives the overhang filter, pillars are
    // placed, and the pad grows from their footprint.
    CubePadPrint run{/*supports*/ true, /*pad*/ true, 0.};

    REQUIRE_NOTHROW(run.print.process());

    const SLAPrintObject &po = run.object();
    REQUIRE_FALSE(po.support_mesh().empty());
    REQUIRE_FALSE(po.pad_mesh().empty());
}

// ── The gateway predicate ─────────────────────────────────────────────────────

TEST_CASE("validate_pad only rejects an empty pad outside embed-object mode",
          "[sla_pad_degradation]")
{
    // The degradation branch sits behind validate_pad() returning false, so the
    // "pad is empty" term of the guard is structurally guaranteed by this
    // predicate. Locking it here means a change to validate_pad() cannot widen
    // what the degradation can swallow without failing a test.
    sla::PadConfig pcfg; // embed_object disabled by default

    REQUIRE_FALSE(pcfg.embed_object.enabled);

    indexed_triangle_set empty_pad;
    REQUIRE(empty_pad.empty());
    REQUIRE_FALSE(validate_pad(empty_pad, pcfg));

    indexed_triangle_set real_pad = its_make_cube(5., 5., 1.);
    REQUIRE_FALSE(real_pad.empty());
    REQUIRE(validate_pad(real_pad, pcfg));

    // Zero-elevation mode accepts an empty pad by design, so the failure branch
    // — and therefore the degradation — is never reached there.
    pcfg.embed_object.enabled = true;
    REQUIRE(validate_pad(empty_pad, pcfg));
}
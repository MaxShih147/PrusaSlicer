#include <catch2/catch_test_macros.hpp>
#include <test_utils.hpp>

#include "libslic3r/SLAPrint.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/FileReader.hpp"

using namespace Slic3r;

// Regression guard for the --import-support-stl pipeline (change
// sla-support-binary-rasterization). The full visual / Z-range behaviour is
// covered by manual end-to-end verification (the project's primary discipline);
// here we lock the public attach contract that drives the whole feature:
//   * single-object scoping + empty-print guard in SLAPrint::attach_imported_support()
//   * the has_imported_support() flag flip that gates the five import-aware steps
//     in SLAPrintSteps.cpp (slice_model Z-clamp, support_points, support_tree,
//     generate_pad, slice_supports).
// Keeping this at the API level (no process()) avoids any geometry/trafo
// flakiness while still failing loudly if the attach wiring regresses.

TEST_CASE("attach_imported_support flips the flag on the object", "[sla_import_support]") {
    SLAPrint print;
    SLAFullPrintConfig fullcfg;

    auto m = FileReader::load_model(TEST_DATA_DIR PATH_SEPARATOR + std::string("20mm_cube") + ".obj");

    fullcfg.printer_technology.setInt(ptSLA);
    fullcfg.set("supports_enable", false); // imported flow never self-generates supports
    fullcfg.set("pad_enable", false);      // raft is part of the imported mesh

    DynamicPrintConfig cfg;
    cfg.apply(fullcfg);

    print.set_status_callback([](const PrintBase::SlicingStatus&) {});
    print.apply(m, cfg);

    REQUIRE_FALSE(print.objects().empty());
    REQUIRE_FALSE(print.objects().front()->has_imported_support());

    indexed_triangle_set support = its_make_cube(5., 5., 5.);
    REQUIRE(print.attach_imported_support(support));
    REQUIRE(print.objects().front()->has_imported_support());
}

TEST_CASE("attach_imported_support is a safe no-op when there is no object", "[sla_import_support]") {
    SLAPrint print; // no apply() -> no SLAPrintObject to attach to

    indexed_triangle_set support = its_make_cube(5., 5., 5.);
    REQUIRE_FALSE(print.attach_imported_support(support));
}
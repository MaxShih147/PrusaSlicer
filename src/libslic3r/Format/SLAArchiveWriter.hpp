///|/ Copyright (c) Prusa Research 2022 - 2023 Tomáš Mészáros @tamasmeszaros
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef SLAARCHIVE_HPP
#define SLAARCHIVE_HPP

#include <stddef.h>
#include <vector>
#include <memory>
#include <string>
#include <cstddef>
#include <functional>

#include "libslic3r/SLA/RasterBase.hpp"
#include "libslic3r/Execution/ExecutionTBB.hpp"
#include "libslic3r/GCode/ThumbnailData.hpp"
#include "libslic3r/Execution/Execution.hpp"

namespace Slic3r {

class SLAPrint;
class SLAPrinterConfig;

// Holds one reusable raster per worker thread, so a raster can serve consecutive
// layers instead of being rebuilt for each one. At 16K a raster owns a ~94 MB
// pixel buffer, so the per-layer shape paid a full-canvas allocation plus two
// passes of zeroing for every layer.
//
// THREAD-BOUND, NOT SHARED, and that is a correctness requirement rather than a
// tuning choice: draw_binary() swaps the rasterizer's gamma LUT and swaps it back,
// which is only safe while no second thread is inside the same raster. See the
// comment above draw_binary() in AGGRaster.hpp.
//
// The TBB container that backs this lives in the .cpp on purpose.
// <tbb/enumerable_thread_specific.h> pulls in <windows.h> on MSVC -- alone among
// the TBB headers this file already needed -- and this header is reached from
// SLAPrint.hpp, so including it here dragged the whole Win32 macro surface into
// every translation unit that touches SLAPrint. Concretely it put the GDI
// ::Polygon function next to Slic3r::Polygon and broke unrelated code that says
// `using namespace Slic3r;`. Keep the include out of the header.
class ThreadBoundRasters {
public:
    using RasterFactory = std::function<std::unique_ptr<sla::RasterBase>()>;

    ThreadBoundRasters();
    ~ThreadBoundRasters();
    ThreadBoundRasters(const ThreadBoundRasters &)            = delete;
    ThreadBoundRasters &operator=(const ThreadBoundRasters &) = delete;

    // This thread's raster: built via `factory` on first use, reset() on every
    // later call so it carries nothing over from the previous layer.
    sla::RasterBase &acquire(const RasterFactory &factory);

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

class SLAArchiveWriter {
protected:
    std::vector<sla::EncodedRaster> m_layers;
    std::vector<sla::EncodedRaster> m_preview_layers;
    double m_preview_scale = 0.;  // 0 = disabled

    virtual std::unique_ptr<sla::RasterBase> create_raster() const = 0;
    virtual sla::RasterEncoder get_encoder() const = 0;

public:
    virtual ~SLAArchiveWriter() = default;

    void set_preview_scale(double s) { m_preview_scale = s; }
    double preview_scale() const { return m_preview_scale; }

    // Fn have to be thread safe: void(sla::RasterBase& raster, size_t lyrid);
    template<class Fn, class CancelFn, class EP = ExecutionTBB>
    void draw_layers(
        size_t     layer_num,
        Fn &&      drawfn,
        CancelFn cancelfn = []() { return false; },
        const EP & ep       = {})
    {
        m_layers.resize(layer_num);
        if (m_preview_scale > 0.)
            m_preview_layers.resize(layer_num);

        // Rasters are reused per worker thread rather than rebuilt per layer; see
        // ThreadBoundRasters above for why the binding to a thread is mandatory.
        // Work stealing does not break it: it moves tasks between threads and the
        // lookup happens on whichever thread actually runs the body, and the body
        // has no nested parallelism that could let a thread re-enter this loop
        // while a layer is half drawn.
        ThreadBoundRasters rasters;
        const ThreadBoundRasters::RasterFactory factory =
            [this] { return create_raster(); };

        execution::for_each(
            ep, size_t(0), m_layers.size(),
            [this, &drawfn, &cancelfn, &rasters, &factory](size_t idx) {
                if (cancelfn()) return;

                sla::RasterBase    &rst = rasters.acquire(factory);
                sla::EncodedRaster &enc = m_layers[idx];
                drawfn(rst, idx);
                enc = rst.encode(get_encoder());
                if (m_preview_scale > 0.)
                    m_preview_layers[idx] = rst.encode(sla::PNGPreviewEncoder{m_preview_scale});
            },
            execution::max_concurrency(ep));
    }

    // Export the print into an archive using the provided filename.
    virtual void export_print(const std::string     fname,
                              const SLAPrint       &print,
                              const ThumbnailsList &thumbnails,
                              const std::string    &projectname = "") = 0;

    // Export preview layers into a separate ZIP file. Returns false if the write
    // failed; never throws on write errors, because a failed preview must not
    // discard an already-exported .sl1.
    bool export_preview_zip(const std::string &fname,
                            const std::string &projectname = "");

    // Factory method to create an archiver instance
    static std::unique_ptr<SLAArchiveWriter> create(
        const std::string &archtype, const SLAPrinterConfig &);
};

} // namespace Slic3r
#endif // SLAARCHIVE_HPP

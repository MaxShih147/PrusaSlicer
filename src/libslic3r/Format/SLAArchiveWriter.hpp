///|/ Copyright (c) Prusa Research 2022 - 2023 Tomáš Mészáros @tamasmeszaros
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef SLAARCHIVE_HPP
#define SLAARCHIVE_HPP

#include <stddef.h>
#include <chrono>
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

// Stage clock accepted by SLAArchiveWriter::draw_layers() (SLA_RASTER_TIMING,
// design D9). A real clock is supplied by the caller that owns the timing state
// (SLAPrint::Steps::rasterize); this empty type only gives the default template
// argument something to name. draw_layers() never calls it: with a null clock
// pointer no steady_clock read happens at all.
struct NoRasterStageClock {
    void add_reset(std::chrono::steady_clock::duration) {}
    void add_encode_layer(std::chrono::steady_clock::duration) {}
    void add_encode_preview(std::chrono::steady_clock::duration) {}
};

class SLAArchiveWriter {
protected:
    std::vector<sla::EncodedRaster> m_layers;
    std::vector<sla::EncodedRaster> m_preview_layers;
    double m_preview_scale = 0.;  // 0 = disabled

    virtual std::unique_ptr<sla::RasterBase> create_raster() const = 0;
    virtual sla::RasterEncoder get_encoder() const = 0;

    // Tile-aware layer encoder, used only when the raster offers a tile map.
    // Empty by default: a format that does not provide one keeps encoding every
    // pixel through get_encoder(), which is what it did before.
    virtual sla::SparseRasterEncoder get_sparse_encoder() const { return {}; }

public:
    virtual ~SLAArchiveWriter() = default;

    void set_preview_scale(double s) { m_preview_scale = s; }
    double preview_scale() const { return m_preview_scale; }

    // Fn have to be thread safe: void(sla::RasterBase& raster, size_t lyrid);
    //
    // stage_clock, when not null, receives the steady_clock duration of the
    // reset (acquire), encode_layer and encode_preview stages, on the worker
    // thread that ran them. It is called from several threads at once and must
    // accumulate without taking a lock.
    template<class Fn, class CancelFn, class EP = ExecutionTBB, class StageClock = NoRasterStageClock>
    void draw_layers(
        size_t     layer_num,
        Fn &&      drawfn,
        CancelFn cancelfn = []() { return false; },
        const EP & ep       = {},
        StageClock *stage_clock = nullptr)
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
            [this, &drawfn, &cancelfn, &rasters, &factory, stage_clock](size_t idx) {
                if (cancelfn()) return;

                using Clock = std::chrono::steady_clock;
                Clock::time_point t;

                if (stage_clock) t = Clock::now();
                sla::RasterBase    &rst = rasters.acquire(factory);
                if (stage_clock) stage_clock->add_reset(Clock::now() - t);

                sla::EncodedRaster &enc = m_layers[idx];
                drawfn(rst, idx);

                // SLA_RASTER_VERIFY=1 only: the layer is final here (model,
                // post-process and support are all drawn), so this is the last
                // point at which a missed tile mark can still be blamed on the
                // drawing rather than on an encoder. Throws on a violation,
                // which aborts the rasterization and fails the CLI run.
                rst.verify_written_tiles(idx);

                if (stage_clock) t = Clock::now();
                // Tile-aware encoding when both halves are available: the format
                // offers a sparse encoder and the raster tracks written tiles.
                // Either one missing falls back to the dense encoder, which
                // reads every pixel and gives the same bytes.
                const sla::SparseRasterEncoder sparse = get_sparse_encoder();
                const sla::TileMap            *tiles  = rst.written_tiles();
                if (sparse && tiles)
                    enc = rst.encode([&sparse, tiles](const void *ptr, size_t w, size_t h,
                                                      size_t num_components) {
                        return sparse(ptr, w, h, num_components, *tiles);
                    });
                else
                    enc = rst.encode(get_encoder());
                if (stage_clock) stage_clock->add_encode_layer(Clock::now() - t);

                if (m_preview_scale > 0.) {
                    if (stage_clock) t = Clock::now();
                    // Same choice as the layer encoder above: skip the clean
                    // tiles when the raster tracks them, otherwise read all.
                    if (tiles) {
                        sla::SparsePNGPreviewEncoder penc{m_preview_scale};
                        m_preview_layers[idx] = rst.encode([&penc, tiles](const void *ptr, size_t w,
                                                                          size_t h, size_t nc) {
                            return penc(ptr, w, h, nc, *tiles);
                        });
                    } else {
                        m_preview_layers[idx] = rst.encode(sla::PNGPreviewEncoder{m_preview_scale});
                    }
                    if (stage_clock) stage_clock->add_encode_preview(Clock::now() - t);
                }
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

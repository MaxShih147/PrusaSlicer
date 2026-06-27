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
#include <atomic>   // [raster-prof]
#include <chrono>   // [raster-prof]
#include <boost/log/trivial.hpp>  // [raster-prof]

#include "libslic3r/SLA/RasterBase.hpp"
#include "libslic3r/Execution/ExecutionTBB.hpp"
#include "libslic3r/GCode/ThumbnailData.hpp"
#include "libslic3r/Execution/Execution.hpp"

namespace Slic3r {

class SLAPrint;
class SLAPrinterConfig;

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
        std::atomic<long long> t_create{0}, t_draw{0}, t_enc{0}, t_prev{0};  // [raster-prof]
        execution::for_each(
            ep, size_t(0), m_layers.size(),
            [this, &drawfn, &cancelfn, &t_create, &t_draw, &t_enc, &t_prev](size_t idx) {
                if (cancelfn()) return;
                using clk = std::chrono::steady_clock;  // [raster-prof]
                auto ns = [](clk::time_point a, clk::time_point b){ return std::chrono::duration_cast<std::chrono::nanoseconds>(b-a).count(); };

                sla::EncodedRaster &enc = m_layers[idx];
                auto t0 = clk::now();
                auto                rst = create_raster();
                auto t1 = clk::now(); t_create += ns(t0,t1);
                drawfn(*rst, idx);
                auto t2 = clk::now(); t_draw += ns(t1,t2);
                enc = rst->encode(get_encoder());
                auto t3 = clk::now(); t_enc += ns(t2,t3);
                if (m_preview_scale > 0.)
                    m_preview_layers[idx] = rst->encode(sla::PNGPreviewEncoder{m_preview_scale});
                auto t4 = clk::now(); t_prev += ns(t3,t4);
            },
            execution::max_concurrency(ep));
        BOOST_LOG_TRIVIAL(error) << "[raster-prof] (summed over threads) create="
            << t_create/1000000 << "ms draw=" << t_draw/1000000 << "ms encode_main="
            << t_enc/1000000 << "ms encode_preview=" << t_prev/1000000 << "ms";
    }

    // Export the print into an archive using the provided filename.
    virtual void export_print(const std::string     fname,
                              const SLAPrint       &print,
                              const ThumbnailsList &thumbnails,
                              const std::string    &projectname = "") = 0;

    // Export preview layers into a separate ZIP file.
    void export_preview_zip(const std::string &fname,
                            const std::string &projectname = "");

    // Factory method to create an archiver instance
    static std::unique_ptr<SLAArchiveWriter> create(
        const std::string &archtype, const SLAPrinterConfig &);
};

} // namespace Slic3r
#endif // SLAARCHIVE_HPP

///|/ Copyright (c) Prusa Research 2020 - 2022 Tomáš Mészáros @tamasmeszaros
///|/ Copyright (c) 2022 ole00 @ole00
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef SLARASTER_CPP
#define SLARASTER_CPP

#include <libslic3r/SLA/RasterBase.hpp>
#include <libslic3r/SLA/AGGRaster.hpp>
// minz image write:
#include <miniz.h>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <cstdlib>

#include "agg/agg_gamma_functions.h"

namespace Slic3r { namespace sla {

EncodedRaster PNGRasterEncoder::operator()(const void *ptr, size_t w, size_t h,
                                           size_t      num_components)
{
    std::vector<uint8_t> buf;
    size_t s = 0;
    
    void *rawdata = tdefl_write_image_to_png_file_in_memory(
        ptr, int(w), int(h), int(num_components), &s);
    
    // On error, data() will return an empty vector. No other info can be
    // retrieved from miniz anyway...
    if (rawdata == nullptr) return EncodedRaster({}, "png");
    
    auto pptr = static_cast<std::uint8_t*>(rawdata);
    
    buf.reserve(s);
    std::copy(pptr, pptr + s, std::back_inserter(buf));
    
    MZ_FREE(rawdata);
    return EncodedRaster(std::move(buf), "png");
}

EncodedRaster PNGPreviewEncoder::operator()(const void *ptr, size_t w, size_t h,
                                             size_t      num_components)
{
    size_t new_w = static_cast<size_t>(w * scale);
    size_t new_h = static_cast<size_t>(h * scale);
    if (new_w == 0) new_w = 1;
    if (new_h == 0) new_h = 1;

    // Box filter downscale
    std::vector<uint8_t> dst(new_w * new_h * num_components, 0);
    double inv_scale = 1.0 / scale;

    for (size_t dy = 0; dy < new_h; ++dy) {
        size_t sy0 = static_cast<size_t>(dy * inv_scale);
        size_t sy1 = static_cast<size_t>((dy + 1) * inv_scale);
        if (sy1 > h) sy1 = h;
        if (sy1 <= sy0) sy1 = sy0 + 1;

        for (size_t dx = 0; dx < new_w; ++dx) {
            size_t sx0 = static_cast<size_t>(dx * inv_scale);
            size_t sx1 = static_cast<size_t>((dx + 1) * inv_scale);
            if (sx1 > w) sx1 = w;
            if (sx1 <= sx0) sx1 = sx0 + 1;

            for (size_t c = 0; c < num_components; ++c) {
                unsigned sum = 0;
                unsigned count = 0;
                for (size_t sy = sy0; sy < sy1; ++sy) {
                    const auto *row = static_cast<const uint8_t *>(ptr) + (sy * w + sx0) * num_components + c;
                    for (size_t sx = sx0; sx < sx1; ++sx) {
                        sum += *row;
                        row += num_components;
                        ++count;
                    }
                }
                dst[(dy * new_w + dx) * num_components + c] = static_cast<uint8_t>(sum / count);
            }
        }
    }

    // Encode downscaled buffer to PNG
    std::vector<uint8_t> buf;
    size_t s = 0;
    void *rawdata = tdefl_write_image_to_png_file_in_memory(
        dst.data(), int(new_w), int(new_h), int(num_components), &s);

    if (rawdata == nullptr) return EncodedRaster({}, "png");

    auto pptr = static_cast<std::uint8_t *>(rawdata);
    buf.reserve(s);
    std::copy(pptr, pptr + s, std::back_inserter(buf));
    MZ_FREE(rawdata);

    return EncodedRaster(std::move(buf), "png");
}

std::ostream &operator<<(std::ostream &stream, const EncodedRaster &bytes)
{
    stream.write(reinterpret_cast<const char *>(bytes.data()),
                 std::streamsize(bytes.size()));
    
    return stream;
}

EncodedRaster PPMRasterEncoder::operator()(const void *ptr, size_t w, size_t h,
                                           size_t      num_components)
{
    std::vector<uint8_t> buf;
    
    auto header = std::string("P5 ") +
            std::to_string(w) + " " +
            std::to_string(h) + " " + "255 ";
    
    auto sz = w * h * num_components;
    size_t s = sz + header.size();
    
    buf.reserve(s);

    auto buff = reinterpret_cast<const std::uint8_t*>(ptr);
    std::copy(header.begin(), header.end(), std::back_inserter(buf));
    std::copy(buff, buff+sz, std::back_inserter(buf));
    
    return EncodedRaster(std::move(buf), "ppm");
}

std::unique_ptr<RasterBase> create_raster_grayscale_aa(
    const Resolution        &res,
    const PixelDim          &pxdim,
    double                   gamma,
    const RasterBase::Trafo &tr)
{
    std::unique_ptr<RasterBase> rst;
    
    if (gamma > 0)
        rst = std::make_unique<RasterGrayscaleAAGammaPower>(res, pxdim, tr, gamma);
    else if (std::abs(gamma - 1.) < 1e-6)
        rst = std::make_unique<RasterGrayscaleAA>(res, pxdim, tr, agg::gamma_none());
    else
        rst = std::make_unique<RasterGrayscaleAA>(res, pxdim, tr, agg::gamma_threshold(.5));
    
    return rst;
}

} // namespace sla
} // namespace Slic3r

#endif // SLARASTER_CPP

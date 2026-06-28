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

// [layer-rle] PRZ V3.0 RLE encoder. Byte-for-byte mirror of
// agent/prz_encoder.py:_rle_encode_layer (row-major over the grayscale samples).
EncodedRaster RLERasterEncoder::operator()(const void *ptr, size_t w, size_t h,
                                           size_t      num_components)
{
    static const uint8_t PRZ_LAYER_HEADER = 0x55;
    static const uint8_t RLE_BLACK = 0x00, RLE_WHITE = 0xC0, RLE_GRAY = 0x40;

    const uint8_t *src    = static_cast<const uint8_t *>(ptr);
    const size_t   n      = w * h;
    const size_t   stride = num_components ? num_components : 1;

    std::vector<uint8_t> out;
    out.reserve(n / 8 + 64);
    out.push_back(PRZ_LAYER_HEADER);

    auto emit_run = [&out](uint8_t value, uint32_t run_len) {
        uint8_t color_type = (value == 0) ? RLE_BLACK
                           : (value == 255) ? RLE_WHITE : RLE_GRAY;
        uint8_t bcb; int extra;
        if      (run_len < 16)      { bcb = 0x00; extra = 0; }
        else if (run_len < 4096)    { bcb = 0x10; extra = 1; }
        else if (run_len < 1048576) { bcb = 0x20; extra = 2; }
        else                        { bcb = 0x30; extra = 3; }
        out.push_back(uint8_t(color_type | bcb | (run_len & 0x0F)));
        if (color_type == RLE_GRAY) out.push_back(value);
        uint32_t shifted = run_len >> 4;             // big-endian extra bytes
        for (int b = extra - 1; b >= 0; --b)
            out.push_back(uint8_t((shifted >> (8 * b)) & 0xFF));
    };

    if (n > 0) {
        uint8_t  cur = src[0];
        uint32_t run = 1;
        for (size_t p = 1; p < n; ++p) {
            uint8_t v = src[p * stride];
            if (v == cur) ++run;
            else { emit_run(cur, run); cur = v; run = 1; }
        }
        emit_run(cur, run);
    }

    uint32_t sum = 0;                                 // checksum excludes header
    for (size_t k = 1; k < out.size(); ++k) sum += out[k];
    out.push_back(uint8_t((~sum) & 0xFF));

    return EncodedRaster(std::move(out), "rle");
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

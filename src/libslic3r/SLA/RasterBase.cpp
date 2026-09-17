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
#include <cstring>

#include <boost/predef/other/endian.h>
#if defined(_MSC_VER) && !defined(__clang__)
#include <intrin.h>
#endif

#include "agg/agg_gamma_functions.h"

namespace Slic3r { namespace sla {

namespace {

// The RLE fast path turns a trailing-zero count into a byte offset, which is only
// valid when the first byte in memory is the least significant one.
#if BOOST_ENDIAN_LITTLE_BYTE
constexpr bool is_little_endian = true;
#else
constexpr bool is_little_endian = false;
#endif

// Loads eight bytes without a pointer cast, so the read is neither misaligned nor
// a strict-aliasing violation. Compilers turn this into a single load.
inline uint64_t load_u64(const uint8_t *p)
{
    uint64_t v;
    std::memcpy(&v, p, sizeof(v));
    return v;
}

// Index of the lowest set bit. x must not be zero.
inline unsigned ctz64(uint64_t x)
{
#if defined(_MSC_VER) && !defined(__clang__)
    unsigned long index;
    _BitScanForward64(&index, x);
    return static_cast<unsigned>(index);
#else
    return static_cast<unsigned>(__builtin_ctzll(x));
#endif
}

} // namespace

bool raster_fastpath_enabled()
{
    // Initialized on first use and never again; C++11 makes the initialization
    // thread-safe, so concurrent encoders on worker threads see one value.
    static const bool enabled = [] {
        const char *value = std::getenv("SLA_RASTER_FASTPATH");
        return !(value != nullptr && std::strcmp(value, "0") == 0);
    }();
    return enabled;
}

bool raster_verify_enabled()
{
    // Read once, like raster_fastpath_enabled(); the workers share the value.
    static const bool enabled = [] {
        const char *value = std::getenv("SLA_RASTER_VERIFY");
        return value != nullptr && std::strcmp(value, "1") == 0;
    }();
    return enabled;
}

TileMap::TileMap(size_t width_px, size_t height_px)
    : m_width_px(width_px)
    , m_height_px(height_px)
    , m_tiles_x((width_px + T - 1) / T)
    , m_tiles_y((height_px + T - 1) / T)
    , m_tile(m_tiles_x * m_tiles_y, uint8_t(0))
    , m_row_any(m_tiles_y, uint8_t(0))
{}

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

// Box filter downscale, generic case: the source block for a destination pixel
// is derived per pixel from the scale, which is what makes any ratio work.
static void preview_box_downscale(const uint8_t *src, size_t w, size_t h,
                                  uint8_t *dst, size_t new_w, size_t new_h,
                                  size_t num_components, double inv_scale)
{
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
                    const auto *row = src + (sy * w + sx0) * num_components + c;
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
}

// Box filter downscale by an exact 1/N. Every destination pixel maps to a fixed
// N x N source block, so the per-pixel boundary arithmetic above collapses into
// loop induction and the divisor becomes loop-invariant. The averaging itself is
// unchanged -- same pixels, same sum, same truncating division -- so this stays
// byte-for-byte equal to the generic path (see the exactness guard at the call
// site for why that equality actually holds).
//
// Reference implementation (design D8): block-major order, kept verbatim.
void preview_box_downscale_integer_reference(const uint8_t *src, size_t w,
                                             uint8_t *dst, size_t new_w, size_t new_h,
                                             size_t num_components, size_t n)
{
    const unsigned area        = static_cast<unsigned>(n * n);
    const size_t   src_stride  = w * num_components;

    for (size_t dy = 0; dy < new_h; ++dy) {
        const uint8_t *block_row = src + (dy * n) * src_stride;
        for (size_t dx = 0; dx < new_w; ++dx) {
            const uint8_t *block = block_row + dx * n * num_components;
            for (size_t c = 0; c < num_components; ++c) {
                unsigned sum = 0;
                const uint8_t *row = block + c;
                for (size_t sy = 0; sy < n; ++sy) {
                    const uint8_t *p = row;
                    for (size_t sx = 0; sx < n; ++sx) {
                        sum += *p;
                        p += num_components;
                    }
                    row += src_stride;
                }
                dst[(dy * new_w + dx) * num_components + c] = static_cast<uint8_t>(sum / area);
            }
        }
    }
}

// Row-major fast path (design D3) for single-channel buffers. Same pixels, same
// unsigned sums, same truncating division as the reference, so the same bytes;
// only the visiting order changes. For every destination row, each of its n
// source rows is walked left to right over columns [0, new_w * n) while the
// per-block totals accumulate in `sums`, and all-zero 8-byte words -- the black
// background -- are skipped whole. A word may straddle two blocks; a zero word
// adds nothing to either, and a non-zero one is attributed byte by byte.
static void preview_box_downscale_integer(const uint8_t *src, size_t w,
                                          uint8_t *dst, size_t new_w, size_t new_h,
                                          size_t num_components, size_t n)
{
    if (num_components != 1 || !raster_fastpath_enabled()) {
        preview_box_downscale_integer_reference(src, w, dst, new_w, new_h, num_components, n);
        return;
    }

    const unsigned area = static_cast<unsigned>(n * n);
    const size_t   span = new_w * n;             // columns read per source row
    std::vector<unsigned> sums(new_w);

    for (size_t dy = 0; dy < new_h; ++dy) {
        std::fill(sums.begin(), sums.end(), 0u);
        const uint8_t *block_row = src + (dy * n) * w;
        for (size_t sy = 0; sy < n; ++sy) {
            const uint8_t *row = block_row + sy * w;
            size_t x = 0;
            // x + 8 <= span keeps every load inside columns [0, new_w * n).
            for (; x + 8 <= span; x += 8) {
                if (load_u64(row + x) == 0) continue;
                // One division per non-zero word; a counter tracks block edges.
                size_t bx = x / n, offset = x - bx * n;
                for (size_t i = x; i < x + 8; ++i) {
                    sums[bx] += row[i];
                    if (++offset == n) { offset = 0; ++bx; }
                }
            }
            for (; x < span; ++x)                    // tail shorter than 8 bytes
                sums[x / n] += row[x];
        }
        for (size_t dx = 0; dx < new_w; ++dx)
            dst[dy * new_w + dx] = static_cast<uint8_t>(sums[dx] / area);
    }
}

// Accumulate one row of target blocks, columns [dx0, dx1), exactly as
// preview_box_downscale_integer() does for a whole row: the same source pixels,
// the same additions, the same truncating division.
static void preview_block_row(const uint8_t *src, size_t w, uint8_t *dst, size_t new_w,
                              size_t dy, size_t dx0, size_t dx1, size_t n,
                              std::vector<unsigned> &sums)
{
    const unsigned area = static_cast<unsigned>(n * n);
    const size_t   xa = dx0 * n, xb = dx1 * n;   // source columns of these blocks

    std::fill(sums.begin() + dx0, sums.begin() + dx1, 0u);
    const uint8_t *block_row = src + (dy * n) * w;
    for (size_t sy = 0; sy < n; ++sy) {
        const uint8_t *row = block_row + sy * w;
        size_t x = xa;
        // x + 8 <= xb keeps every load inside the columns of these blocks.
        for (; x + 8 <= xb; x += 8) {
            if (load_u64(row + x) == 0) continue;
            size_t bx = x / n, offset = x - bx * n;
            for (size_t i = x; i < x + 8; ++i) {
                sums[bx] += row[i];
                if (++offset == n) { offset = 0; ++bx; }
            }
        }
        for (; x < xb; ++x)                      // tail shorter than 8 bytes
            sums[x / n] += row[x];
    }
    for (size_t dx = dx0; dx < dx1; ++dx)
        dst[dy * new_w + dx] = static_cast<uint8_t>(sums[dx] / area);
}

// Tile-aware downscale (design D6 (4)). Requires T % n == 0, so every n x n
// source block lies inside a single tile: a block in a clean tile is all
// background and averages to 0, which is what dst already holds, so those
// blocks are never read. Blocks in written tiles go through the same
// accumulation as the dense fast path, hence the same bytes.
static void preview_box_downscale_integer_sparse(const uint8_t *src, size_t w,
                                                 uint8_t *dst, size_t new_w, size_t new_h,
                                                 size_t num_components, size_t n,
                                                 const TileMap &tiles)
{
    if (num_components != 1 || !raster_fastpath_enabled() || TileMap::T % n != 0 ||
        tiles.width_px() != w) {
        preview_box_downscale_integer(src, w, dst, new_w, new_h, num_components, n);
        return;
    }

    const size_t blocks_per_tile = TileMap::T / n;   // target pixels per tile side
    std::vector<unsigned> sums(new_w);

    for (size_t ty = 0; ty < tiles.tiles_y(); ++ty) {
        const size_t dy0 = ty * blocks_per_tile;
        if (dy0 >= new_h) break;                     // clipped by new_h (D7)
        if (!tiles.row_written(ty)) continue;        // clean row stays zero
        const size_t dy1 = std::min(dy0 + blocks_per_tile, new_h);

        tiles.for_each_written_span(ty, [&](TileMap::PixelSpan cols) {
            const size_t dx0 = cols.begin / n;       // cols.begin is a multiple of T
            // Only whole blocks: a span clipped to w ends at a block boundary or
            // past new_w * n, and the columns beyond new_w * n are never read.
            const size_t dx1 = std::min(cols.end / n, new_w);
            if (dx0 >= dx1) return;
            for (size_t dy = dy0; dy < dy1; ++dy)
                preview_block_row(src, w, dst, new_w, dy, dx0, dx1, n, sums);
        });
    }
}

// Shared body of the two preview encoders: tiles == nullptr reads every pixel.
static EncodedRaster encode_preview_png(const void *ptr, size_t w, size_t h,
                                        size_t num_components, double scale,
                                        const TileMap *tiles)
{
    size_t new_w = static_cast<size_t>(w * scale);
    size_t new_h = static_cast<size_t>(h * scale);
    if (new_w == 0) new_w = 1;
    if (new_h == 0) new_h = 1;

    std::vector<uint8_t> dst(new_w * new_h * num_components, 0);
    const auto  *src       = static_cast<const uint8_t *>(ptr);
    const double inv_scale = 1.0 / scale;

    // Take the fixed-block path only when 1/scale is EXACTLY an integer. The
    // generic path derives its block bounds from `dy * inv_scale`, and for an
    // exact integer that product is representable to the last bit, so both paths
    // land on the same blocks and emit the same bytes. Accepting a near-integer
    // would break that: a reciprocal a hair under N truncates a row early on some
    // dy and silently shifts the output. The trailing bounds checks cover the case
    // where the source is not a whole number of blocks wide or tall -- there the
    // generic path clamps the last block and averages over fewer pixels, which the
    // fixed-block path cannot reproduce.
    const size_t n = static_cast<size_t>(inv_scale);
    const bool   fixed_block = n >= 1 &&
                               inv_scale == static_cast<double>(n) &&
                               new_w * n <= w && new_h * n <= h;

    if (fixed_block && tiles != nullptr)
        preview_box_downscale_integer_sparse(src, w, dst.data(), new_w, new_h,
                                             num_components, n, *tiles);
    else if (fixed_block)
        preview_box_downscale_integer(src, w, dst.data(), new_w, new_h, num_components, n);
    else
        preview_box_downscale(src, w, h, dst.data(), new_w, new_h, num_components, inv_scale);

    // Encode downscaled buffer to PNG. Level 1 rather than the level 6 that the
    // plain tdefl_write_image_to_png_file_in_memory() hardcodes: preview layers are
    // overwhelmingly black, so the extra levels buy very little size for several
    // times the encoding time. Only the preview goes through here -- the .sl1 layer
    // encoders above are untouched.
    std::vector<uint8_t> buf;
    size_t s = 0;
    void *rawdata = tdefl_write_image_to_png_file_in_memory_ex(
        dst.data(), int(new_w), int(new_h), int(num_components), &s, 1, MZ_FALSE);

    if (rawdata == nullptr) return EncodedRaster({}, "png");

    auto pptr = static_cast<std::uint8_t *>(rawdata);
    buf.reserve(s);
    std::copy(pptr, pptr + s, std::back_inserter(buf));
    MZ_FREE(rawdata);

    return EncodedRaster(std::move(buf), "png");
}

EncodedRaster PNGPreviewEncoder::operator()(const void *ptr, size_t w, size_t h,
                                            size_t      num_components)
{
    return encode_preview_png(ptr, w, h, num_components, scale, nullptr);
}

EncodedRaster SparsePNGPreviewEncoder::operator()(const void *ptr, size_t w, size_t h,
                                                  size_t         num_components,
                                                  const TileMap &tiles)
{
    return encode_preview_png(ptr, w, h, num_components, scale, &tiles);
}

// RLE fast path (design D2) for single-channel buffers on little-endian targets.
// It yields the same maximal same-value runs as rle_encode_reference(), hence the
// same bytes: eight bytes at a time are compared against the current run value,
// and on a mismatch the trailing-zero count locates the first differing byte.
// Header, run emission and checksum are the reference's, unchanged.
static EncodedRaster rle_encode_swar(const uint8_t *src, size_t n)
{
    static const uint8_t PRZ_LAYER_HEADER = 0x55;
    static const uint8_t RLE_BLACK = 0x00, RLE_WHITE = 0xC0, RLE_GRAY = 0x40;

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
        const uint64_t ones = 0x0101010101010101ull;
        uint8_t  cur = src[0];
        uint32_t run = 0;
        size_t   p   = 0;
        // p + 8 <= n keeps every load inside the buffer; no padding is needed.
        while (p + 8 <= n) {
            const uint64_t x = load_u64(src + p) ^ (uint64_t(cur) * ones);
            if (x == 0) { run += 8; p += 8; continue; }
            // x != 0, so k <= 7 and src[p + k] is still within this word.
            const size_t k = ctz64(x) >> 3;          // bytes still equal to cur
            run += uint32_t(k);
            p   += k;
            emit_run(cur, run);
            cur = src[p];
            run = 1;
            ++p;
        }
        for (; p < n; ++p) {                         // tail shorter than 8 bytes
            uint8_t v = src[p];
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

// [layer-rle] Tile-aware variant of rle_encode_swar(): same run decomposition,
// but the clean stretches of a row are added to the current run by arithmetic
// (they are all background) instead of being read. Written spans go through the
// same word scan, and the run state is never flushed at a row or span boundary,
// so the sequence of maximal same-value runs -- and therefore every output byte
// -- matches the dense encoders (design D1, D6 ③).
static EncodedRaster rle_encode_sparse(const uint8_t *src, size_t w, size_t h,
                                       const TileMap &tiles)
{
    static const uint8_t PRZ_LAYER_HEADER = 0x55;
    static const uint8_t RLE_BLACK = 0x00, RLE_WHITE = 0xC0, RLE_GRAY = 0x40;
    static const uint8_t BACKGROUND = 0;

    assert(tiles.width_px() == w && tiles.height_px() == h);

    std::vector<uint8_t> out;
    out.reserve(w * h / 64 + 64);
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

    // The run being built. It spans spans and rows exactly as the dense walk's
    // does; open is false only before the very first pixel.
    bool     open = false;
    uint8_t  cur  = 0;
    uint32_t run  = 0;

    // Charge count background pixels without looking at them.
    auto append_background = [&](size_t count) {
        if (count == 0) return;
        if (!open) { open = true; cur = BACKGROUND; run = 0; }
        else if (cur != BACKGROUND) { emit_run(cur, run); cur = BACKGROUND; run = 0; }
        run += uint32_t(count);
    };

    // Word scan of a written span, identical to rle_encode_swar()'s loop.
    const uint64_t ones = 0x0101010101010101ull;
    auto append_pixels = [&](const uint8_t *s, size_t n) {
        size_t p = 0;
        if (!open) { open = true; cur = s[0]; run = 0; }
        while (p + 8 <= n) {
            const uint64_t x = load_u64(s + p) ^ (uint64_t(cur) * ones);
            if (x == 0) { run += 8; p += 8; continue; }
            const size_t k = ctz64(x) >> 3;          // bytes still equal to cur
            run += uint32_t(k);
            p   += k;
            emit_run(cur, run);
            cur = s[p];
            run = 1;
            ++p;
        }
        for (; p < n; ++p) {                         // tail shorter than 8 bytes
            uint8_t v = s[p];
            if (v == cur) ++run;
            else { emit_run(cur, run); cur = v; run = 1; }
        }
    };

    for (size_t ty = 0; ty < tiles.tiles_y(); ++ty) {
        const TileMap::PixelSpan rows = tiles.pixel_rows(ty);
        if (!tiles.row_written(ty)) {                // whole tile row is clean
            append_background((rows.end - rows.begin) * w);
            continue;
        }
        for (size_t y = rows.begin; y < rows.end; ++y) {
            size_t x = 0;                            // first column not emitted yet
            tiles.for_each_written_span(ty, [&](TileMap::PixelSpan cols) {
                append_background(cols.begin - x);   // clean gap before this span
                append_pixels(src + y * w + cols.begin, cols.end - cols.begin);
                x = cols.end;
            });
            append_background(w - x);                // clean tail of the row
        }
    }
    if (open) emit_run(cur, run);

    uint32_t sum = 0;                                 // checksum excludes header
    for (size_t k = 1; k < out.size(); ++k) sum += out[k];
    out.push_back(uint8_t((~sum) & 0xFF));

    return EncodedRaster(std::move(out), "rle");
}

EncodedRaster SparseRLERasterEncoder::operator()(const void *ptr, size_t w, size_t h,
                                                 size_t         num_components,
                                                 const TileMap &tiles)
{
    if (num_components != 1 || !is_little_endian || !raster_fastpath_enabled())
        return rle_encode_reference(ptr, w, h, num_components);

    return rle_encode_sparse(static_cast<const uint8_t *>(ptr), w, h, tiles);
}

// [layer-rle] PRZ V3.0 RLE encoder. Byte-for-byte mirror of
// agent/prz_encoder.py:_rle_encode_layer (row-major over the grayscale samples).
EncodedRaster RLERasterEncoder::operator()(const void *ptr, size_t w, size_t h,
                                           size_t      num_components)
{
    if (num_components != 1 || !is_little_endian || !raster_fastpath_enabled())
        return rle_encode_reference(ptr, w, h, num_components);

    return rle_encode_swar(static_cast<const uint8_t *>(ptr), w * h);
}

// Reference implementation (design D8): the byte-by-byte walk, kept verbatim.
EncodedRaster rle_encode_reference(const void *ptr, size_t w, size_t h,
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
    const RasterBase::Trafo &tr,
    RasterPostProcessor      pp,
    bool                     zero_preserving_pixel_local)
{
    std::unique_ptr<RasterBase> rst;
    RasterPostProcess post{std::move(pp), zero_preserving_pixel_local};

    if (gamma > 0)
        rst = std::make_unique<RasterGrayscaleAAGammaPower>(res, pxdim, tr, gamma, std::move(post));
    else if (std::abs(gamma - 1.) < 1e-6)
        rst = std::make_unique<RasterGrayscaleAA>(res, pxdim, tr, agg::gamma_none(), std::move(post));
    else
        rst = std::make_unique<RasterGrayscaleAA>(res, pxdim, tr, agg::gamma_threshold(.5), std::move(post));

    return rst;
}

} // namespace sla
} // namespace Slic3r

#endif // SLARASTER_CPP

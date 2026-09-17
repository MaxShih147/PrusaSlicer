// Differential tests for the raster scan fast paths (design D10).
//
// Every fast path is checked byte for byte against the reference implementation
// it replaces: the RLE word-at-a-time run scan against rle_encode_reference(),
// and the row-major preview downscale against
// preview_box_downscale_integer_reference(). The reference functions are the old
// code kept verbatim, so equality here means the output did not change.

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/libslic3r.h"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/PNGReadWrite.hpp"
#include "libslic3r/SLA/AGGRaster.hpp"
#include "libslic3r/SLA/RasterBase.hpp"

// Page protection for the guard page tests of the fixed-block preview kernel.
// Last, so the Windows macros cannot reach the headers above.
#if defined(_WIN32)
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <windows.h>
#elif defined(__unix__) || defined(__APPLE__)
#  include <sys/mman.h>
#  include <unistd.h>
#endif

using namespace Slic3r;

namespace {

constexpr size_t NoDifference = std::numeric_limits<size_t>::max();

// Index of the first differing byte, or NoDifference. Catch would print both
// vectors in full on a plain `a == b` failure, which is useless at 16K sizes.
size_t first_difference(const std::vector<uint8_t> &a, const std::vector<uint8_t> &b)
{
    const size_t common = std::min(a.size(), b.size());
    for (size_t i = 0; i < common; ++i)
        if (a[i] != b[i]) return i;
    return a.size() == b.size() ? NoDifference : common;
}

std::vector<uint8_t> bytes_of(const sla::EncodedRaster &enc)
{
    const auto *p = static_cast<const uint8_t *>(enc.data());
    return {p, p + enc.size()};
}

struct Run { uint8_t value; uint32_t length; };

// Independent decoder of the PRZ layer RLE, used to check that a case really
// contains the runs it is meant to exercise and that the checksum is valid.
std::vector<Run> decode_rle_runs(const std::vector<uint8_t> &enc)
{
    REQUIRE(enc.size() >= 2);
    REQUIRE(enc.front() == 0x55);

    uint32_t sum = 0;
    for (size_t k = 1; k + 1 < enc.size(); ++k) sum += enc[k];
    REQUIRE(enc.back() == uint8_t((~sum) & 0xFF));

    std::vector<Run> runs;
    const size_t end = enc.size() - 1;
    size_t i = 1;
    while (i < end) {
        const uint8_t b     = enc[i++];
        const uint8_t color = b & 0xC0;
        REQUIRE(color != 0x80);
        uint8_t value = color == 0xC0 ? 255 : 0;
        if (color == 0x40) {
            REQUIRE(i < end);
            value = enc[i++];
        }
        uint32_t high = 0;
        for (int extra = (b >> 4) & 0x03; extra > 0; --extra) {
            REQUIRE(i < end);
            high = (high << 8) | enc[i++];
        }
        runs.push_back({value, (high << 4) | uint32_t(b & 0x0F)});
    }
    return runs;
}

size_t total_length(const std::vector<Run> &runs)
{
    size_t total = 0;
    for (const Run &r : runs) total += r.length;
    return total;
}

bool has_run(const std::vector<Run> &runs, uint8_t value, uint32_t length)
{
    for (const Run &r : runs)
        if (r.value == value && r.length == length) return true;
    return false;
}

// Encodes with RLERasterEncoder (the dispatching entry point) and with the
// reference, requires identical bytes, and returns the decoded runs.
std::vector<Run> require_rle_matches_reference(const uint8_t *px, size_t w, size_t h,
                                               size_t num_components = 1)
{
    INFO("w = " << w << ", h = " << h << ", num_components = " << num_components);

    const std::vector<uint8_t> fast = bytes_of(sla::RLERasterEncoder{}(px, w, h, num_components));
    const std::vector<uint8_t> ref  = bytes_of(sla::rle_encode_reference(px, w, h, num_components));

    REQUIRE(fast.size() == ref.size());
    REQUIRE(first_difference(fast, ref) == NoDifference);

    std::vector<Run> runs = decode_rle_runs(ref);
    REQUIRE(total_length(runs) == w * h);
    return runs;
}

// Downscales through PNGPreviewEncoder and decodes the PNG, then compares the
// pixels with the reference. The REQUIREs on scale and size mirror the
// fixed-block guard in PNGPreviewEncoder, so a case that would silently fall to
// the generic path fails here instead of comparing the wrong code.
void require_preview_matches_reference(const std::vector<uint8_t> &src, size_t w, size_t h,
                                       size_t n)
{
    INFO("w = " << w << ", h = " << h << ", n = " << n);
    REQUIRE(src.size() == w * h);

    const double scale = 1.0 / double(n);
    REQUIRE(1.0 / scale == double(n));

    const size_t new_w = static_cast<size_t>(w * scale);
    const size_t new_h = static_cast<size_t>(h * scale);
    REQUIRE(new_w >= 1);
    REQUIRE(new_h >= 1);
    REQUIRE(new_w * n <= w);
    REQUIRE(new_h * n <= h);

    const sla::EncodedRaster enc = sla::PNGPreviewEncoder{scale}(src.data(), w, h, 1);
    png::ImageGreyscale img;
    REQUIRE(png::decode_png({enc.data(), enc.size()}, img));
    REQUIRE(img.cols == new_w);
    REQUIRE(img.rows == new_h);

    std::vector<uint8_t> expected(new_w * new_h);
    sla::preview_box_downscale_integer_reference(src.data(), w, expected.data(), new_w, new_h, 1, n);

    REQUIRE(first_difference(img.buf, expected) == NoDifference);
}

// Anti-aliased grayscale raster with a circle and a rotated square, so the
// buffer holds real AA gray edges rather than synthetic ones.
std::vector<uint8_t> antialiased_pixels(size_t w, size_t h)
{
    const double mm_per_px = 0.05;
    const double disp_w = w * mm_per_px, disp_h = h * mm_per_px;

    sla::Resolution res{w, h};
    sla::PixelDim   pixdim{mm_per_px, mm_per_px};
    sla::RasterGrayscaleAAGammaPower raster(res, pixdim, {}, 1.);

    const BoundingBox bb({0, 0}, {scaled(disp_w), scaled(disp_h)});
    const double cx = bb.center().x(), cy = bb.center().y();

    auto polygon = [&](double radius_mm, int vertices, double phase) {
        ExPolygon poly;
        const double r = scaled(radius_mm);
        for (int i = 0; i < vertices; ++i) {
            const double a = phase + 2. * PI * i / vertices;
            poly.contour.points.emplace_back(coord_t(cx + r * std::cos(a)),
                                             coord_t(cy + r * std::sin(a)));
        }
        return poly;
    };

    ExPolygon square = polygon(0.4 * std::min(disp_w, disp_h), 4, 0.3);
    square.translate(scaled(-0.2 * disp_w), 0);
    raster.draw(square);

    ExPolygon circle = polygon(0.3 * std::min(disp_w, disp_h), 90, 0.);
    circle.translate(scaled(0.2 * disp_w), 0);
    raster.draw(circle);

    std::vector<uint8_t> pixels;
    raster.encode([&pixels](const void *ptr, size_t pw, size_t ph, size_t nc) {
        REQUIRE(nc == 1);
        const auto *p = static_cast<const uint8_t *>(ptr);
        pixels.assign(p, p + pw * ph);
        return sla::EncodedRaster{};
    });
    REQUIRE(pixels.size() == w * h);
    return pixels;
}

// ---------------------------------------------------------------------------
// Tile map helpers (design D4/D5).
//
// The rasters below use 0.05 mm per pixel, so a polygon given in pixels lands
// on exact pixel boundaries. The default trafo mirrors y, hence the (h - py).
// ---------------------------------------------------------------------------

constexpr double TileMmPerPx = 0.05;

ExPolygon pixel_polygon(size_t h, const std::vector<std::pair<double, double>> &pts)
{
    ExPolygon poly;
    for (const auto &pt : pts)
        poly.contour.points.emplace_back(coord_t(scaled(pt.first * TileMmPerPx)),
                                         coord_t(scaled((double(h) - pt.second) * TileMmPerPx)));
    return poly;
}

// Rectangle covering pixels [x0, x1) x [y0, y1).
ExPolygon pixel_rect(size_t h, double x0, double y0, double x1, double y1)
{
    return pixel_polygon(h, {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}});
}

std::vector<uint8_t> raster_pixels(const sla::RasterBase &raster, size_t w, size_t h)
{
    std::vector<uint8_t> pixels;
    raster.encode([&pixels](const void *ptr, size_t pw, size_t ph, size_t nc) {
        REQUIRE(nc == 1);
        const auto *p = static_cast<const uint8_t *>(ptr);
        pixels.assign(p, p + pw * ph);
        return sla::EncodedRaster{};
    });
    REQUIRE(pixels.size() == w * h);
    return pixels;
}

using TileList = std::vector<std::pair<size_t, size_t>>;

// Builder for the expected lists: the commas stay inside these parentheses,
// where the preprocessor leaves them alone.
TileList tile_list(std::initializer_list<std::pair<size_t, size_t>> tiles)
{
    return TileList(tiles.begin(), tiles.end());
}

TileList marked_tiles(const sla::TileMap &tiles)
{
    TileList out;
    for (size_t ty = 0; ty < tiles.tiles_y(); ++ty)
        for (size_t tx = 0; tx < tiles.tiles_x(); ++tx)
            if (tiles.written(tx, ty)) out.emplace_back(tx, ty);
    return out;
}

// Invariant I2: a pixel that was written lies in a marked tile, and that tile's
// row summary is set too -- a consumer that trusts the summary skips the whole
// row otherwise.
void require_marks_cover_pixels(const std::vector<uint8_t> &pixels, size_t w, size_t h,
                                const sla::TileMap &tiles)
{
    const size_t T = sla::TileMap::T;
    TileList unmarked_tile, unmarked_row;

    for (size_t y = 0; y < h; ++y)
        for (size_t x = 0; x < w; ++x) {
            if (pixels[y * w + x] == 0) continue;
            const size_t tx = x / T, ty = y / T;
            if (!tiles.written(tx, ty) && unmarked_tile.size() < 8)
                unmarked_tile.emplace_back(x, y);
            if (!tiles.row_written(ty) && unmarked_row.size() < 8)
                unmarked_row.emplace_back(x, y);
        }

    REQUIRE(unmarked_tile.empty());   // written pixel in a tile nobody marked
    REQUIRE(unmarked_row.empty());    // marked tile in a row whose summary is clear
}

// No marked tile may lie outside the pixel box the drawing could have touched,
// widened by one pixel on each side: AGG can hand the adapter a span end with
// zero coverage, which marks without writing, but never reaches further.
void require_marks_within(const sla::TileMap &tiles, size_t w, size_t h,
                          size_t x0, size_t y0, size_t x1, size_t y1)
{
    const size_t T = sla::TileMap::T;
    const size_t tx_lo = (x0 > 0 ? x0 - 1 : 0) / T;
    const size_t tx_hi = std::min(x1, w - 1) / T;
    const size_t ty_lo = (y0 > 0 ? y0 - 1 : 0) / T;
    const size_t ty_hi = std::min(y1, h - 1) / T;

    TileList outside;
    for (const auto &t : marked_tiles(tiles))
        if ((t.first < tx_lo || t.first > tx_hi || t.second < ty_lo || t.second > ty_hi) &&
            outside.size() < 8)
            outside.push_back(t);

    REQUIRE(outside.empty());
}

// SLA_RASTER_FASTPATH=0 hides the tile map, so there is nothing for a tile
// test to look at. Say so and return rather than fail on a null map.
bool tile_tracking_available()
{
    if (sla::raster_fastpath_enabled()) return true;
    WARN("Tile tracking disabled via FASTPATH=0; skipping.");
    return false;
}

// Targeted form of require_marks_cover_pixels for canvases where a full scan
// would be tens of millions of pixels: only the window [x0, x1) x [y0, y1) is
// read, through read_pixel, so the whole check stays in the millisecond range.
void require_window_marks(const sla::RasterGrayscaleAA &raster, const sla::TileMap &tiles,
                          size_t x0, size_t y0, size_t x1, size_t y1)
{
    const size_t T = sla::TileMap::T;
    TileList unmarked_tile, unmarked_row;

    for (size_t y = y0; y < y1; ++y)
        for (size_t x = x0; x < x1; ++x) {
            if (raster.read_pixel(x, y) == 0) continue;
            const size_t tx = x / T, ty = y / T;
            if (!tiles.written(tx, ty) && unmarked_tile.size() < 8)
                unmarked_tile.emplace_back(x, y);
            if (!tiles.row_written(ty) && unmarked_row.size() < 8)
                unmarked_row.emplace_back(x, y);
        }

    REQUIRE(unmarked_tile.empty());
    REQUIRE(unmarked_row.empty());
}

// Pixel range of one tile, as every consumer computes it (design D7).
void require_tile_span(const sla::TileMap &tiles, size_t tx, size_t ty,
                       size_t x0, size_t x1, size_t y0, size_t y1)
{
    const sla::TileMap::PixelSpan cols = tiles.pixel_columns(tx, tx + 1);
    const sla::TileMap::PixelSpan rows = tiles.pixel_rows(ty);
    REQUIRE(cols.begin == x0);
    REQUIRE(cols.end == x1);
    REQUIRE(rows.begin == y0);
    REQUIRE(rows.end == y1);
}

// Post-process that records how it was called, so a test can tell the sparse
// path (one call per row segment) from the whole-buffer one (a single call).
struct PostProcLog {
    struct Call { size_t w, h, nc; };
    std::vector<Call> calls;
    size_t pixels_seen = 0;
};

sla::RasterPostProcess make_postproc(std::shared_ptr<PostProcLog> log, bool zero_preserving,
                                     std::function<void(uint8_t *, size_t, size_t)> body)
{
    sla::RasterPostProcess post;
    post.zero_preserving_pixel_local = zero_preserving;
    post.fn = [log, body](void *ptr, size_t w, size_t h, size_t nc) {
        log->calls.push_back({w, h, nc});
        log->pixels_seen += w * h * nc;
        body(static_cast<uint8_t *>(ptr), w, h);
    };
    return post;
}

// Pixel-local and zero-preserving, like SL1's AA quantization with blur = 0.
void map_nonzero_to(uint8_t *p, size_t w, size_t h, uint8_t value)
{
    for (size_t i = 0; i < w * h; ++i)
        if (p[i] != 0) p[i] = value;
}

// Pixels the sparse path would hand to the post-process: the written spans of
// every tile row, full tile height.
size_t written_span_pixels(const sla::TileMap &tiles)
{
    size_t total = 0;
    for (size_t ty = 0; ty < tiles.tiles_y(); ++ty) {
        const sla::TileMap::PixelSpan rows = tiles.pixel_rows(ty);
        tiles.for_each_written_span(ty, [&](sla::TileMap::PixelSpan cols) {
            total += (cols.end - cols.begin) * (rows.end - rows.begin);
        });
    }
    return total;
}

// One layer's drawing, so a reused raster and a freshly built one can be given
// exactly the same geometry.
using LayerDraw = std::function<void(sla::RasterGrayscaleAA &)>;

// Scattered shapes that reach the right and bottom edge tiles of a 400 x 240
// canvas, including partial coverage at the very last pixel.
void draw_scattered(sla::RasterGrayscaleAA &raster, size_t h)
{
    raster.draw_binary(pixel_rect(h, 0, 0, 3, 3));            // origin tile
    raster.draw_binary(pixel_rect(h, 150, 90, 260, 140));     // spans four tiles
    raster.draw_binary(pixel_rect(h, 399, 239, 400, 240));    // far corner pixel
    raster.draw(pixel_polygon(h, {{78.5, 30.25}, {120.75, 70.5}, {60.25, 95.5}}));
}

// Every tile row summary agrees with the tiles in that row.
void require_row_summaries_consistent(const sla::TileMap &tiles)
{
    std::vector<size_t> wrong;
    for (size_t ty = 0; ty < tiles.tiles_y(); ++ty) {
        bool any = false;
        for (size_t tx = 0; tx < tiles.tiles_x(); ++tx)
            any = any || tiles.written(tx, ty);
        if (any != tiles.row_written(ty) && wrong.size() < 8) wrong.push_back(ty);
    }
    REQUIRE(wrong.empty());
}

// ---------------------------------------------------------------------------
// Sparse RLE helpers (design D6 (3), tasks 2.13).
// ---------------------------------------------------------------------------

// Value put into every pixel the sparse encoder must never read. It is neither
// black nor white, so a clean area that gets scanned instead of counted changes
// the output bytes and the comparison below fails.
constexpr uint8_t CleanAreaPoison = 0xAA;

// Copy of `px` with every pixel outside the written spans of `tiles` replaced by
// the poison value. Requires invariant I2 to hold, so check it first when the
// map came from a real raster.
std::vector<uint8_t> poison_clean_areas(const std::vector<uint8_t> &px, size_t w, size_t h,
                                        const sla::TileMap &tiles)
{
    REQUIRE(px.size() == w * h);
    std::vector<uint8_t> out(px.size(), CleanAreaPoison);

    for (size_t ty = 0; ty < tiles.tiles_y(); ++ty) {
        if (!tiles.row_written(ty)) continue;
        const sla::TileMap::PixelSpan rows = tiles.pixel_rows(ty);
        tiles.for_each_written_span(ty, [&](sla::TileMap::PixelSpan cols) {
            const size_t len = cols.end - cols.begin;
            for (size_t y = rows.begin; y < rows.end; ++y) {
                const uint8_t *src = px.data() + y * w + cols.begin;
                std::copy(src, src + len, out.data() + y * w + cols.begin);
            }
        });
    }
    return out;
}

size_t count_runs(const std::vector<Run> &runs, uint8_t value, uint32_t length)
{
    size_t n = 0;
    for (const Run &r : runs)
        if (r.value == value && r.length == length) ++n;
    return n;
}

// Sets pixels [x0, x1) of row y and marks the tiles they touch, the way the
// pixel format adapter does during drawing.
void write_span(std::vector<uint8_t> &px, sla::TileMap &tiles, size_t w,
                size_t y, size_t x0, size_t x1, uint8_t value)
{
    for (size_t x = x0; x < x1; ++x) px[y * w + x] = value;
    tiles.mark_span(x0, y, x1 - x0);
}

// The byte-for-byte check. `sparse_src` is the poisoned copy and `dense_src` the
// buffer as it really is: equal output proves the clean areas were charged to
// the run length by arithmetic and never read.
std::vector<Run> require_sparse_rle_matches(const std::vector<uint8_t> &sparse_src,
                                            const std::vector<uint8_t> &dense_src,
                                            size_t w, size_t h, const sla::TileMap &tiles)
{
    INFO("w = " << w << ", h = " << h);
    REQUIRE(sparse_src.size() == w * h);
    REQUIRE(dense_src.size() == w * h);
    REQUIRE(tiles.width_px() == w);
    REQUIRE(tiles.height_px() == h);

    const std::vector<uint8_t> sparse =
        bytes_of(sla::SparseRLERasterEncoder{}(sparse_src.data(), w, h, 1, tiles));
    const std::vector<uint8_t> dense =
        bytes_of(sla::RLERasterEncoder{}(dense_src.data(), w, h, 1));
    const std::vector<uint8_t> ref =
        bytes_of(sla::rle_encode_reference(dense_src.data(), w, h, 1));

    REQUIRE(dense.size() == ref.size());
    REQUIRE(first_difference(dense, ref) == NoDifference);     // the oracle itself
    REQUIRE(sparse.size() == dense.size());
    REQUIRE(first_difference(sparse, dense) == NoDifference);  // 100% byte identical

    std::vector<Run> runs = decode_rle_runs(sparse);
    REQUIRE(total_length(runs) == w * h);
    return runs;
}

// ---------------------------------------------------------------------------
// Sparse preview helpers (design D6 (4), tasks 2.14).
//
// preview_box_downscale_integer() and preview_box_downscale_integer_sparse()
// are static in RasterBase.cpp, so the tests drive them through the two public
// encoders that dispatch to them -- PNGPreviewEncoder for the dense path and
// SparsePNGPreviewEncoder for the sparse one -- and use the public
// preview_box_downscale_integer_reference() as the oracle.
// ---------------------------------------------------------------------------

sla::EncodedRaster dense_preview_png(const std::vector<uint8_t> &src, size_t w, size_t h,
                                     size_t n)
{
    return sla::PNGPreviewEncoder{1.0 / double(n)}(src.data(), w, h, 1);
}

sla::EncodedRaster sparse_preview_png(const std::vector<uint8_t> &src, size_t w, size_t h,
                                      size_t n, const sla::TileMap &tiles)
{
    return sla::SparsePNGPreviewEncoder{1.0 / double(n)}(src.data(), w, h, 1, tiles);
}

// Target size the encoders derive for factor n, by the same arithmetic.
void preview_size(size_t w, size_t h, size_t n, size_t &new_w, size_t &new_h)
{
    const double scale = 1.0 / double(n);
    new_w = static_cast<size_t>(w * scale);
    new_h = static_cast<size_t>(h * scale);
    if (new_w == 0) new_w = 1;
    if (new_h == 0) new_h = 1;
}

std::vector<uint8_t> decode_preview(const sla::EncodedRaster &enc, size_t new_w, size_t new_h)
{
    png::ImageGreyscale img;
    REQUIRE(png::decode_png({enc.data(), enc.size()}, img));
    REQUIRE(img.cols == new_w);
    REQUIRE(img.rows == new_h);
    REQUIRE(img.buf.size() == new_w * new_h);
    return img.buf;
}

// Target pixels outside the box [dx0, dx1) x [dy0, dy1) that are not background.
size_t nonzero_outside_box(const std::vector<uint8_t> &prev, size_t new_w, size_t new_h,
                           size_t dx0, size_t dy0, size_t dx1, size_t dy1)
{
    size_t n = 0;
    for (size_t dy = 0; dy < new_h; ++dy)
        for (size_t dx = 0; dx < new_w; ++dx) {
            if (dx >= dx0 && dx < dx1 && dy >= dy0 && dy < dy1) continue;
            if (prev[dy * new_w + dx] != 0) ++n;
        }
    return n;
}

size_t count_value_in_box(const std::vector<uint8_t> &prev, size_t new_w,
                          size_t dx0, size_t dy0, size_t dx1, size_t dy1, uint8_t value)
{
    size_t n = 0;
    for (size_t dy = dy0; dy < dy1; ++dy)
        for (size_t dx = dx0; dx < dx1; ++dx)
            if (prev[dy * new_w + dx] == value) ++n;
    return n;
}

// Three-way check of one downscale factor: the reference downscale of the real
// canvas, the dense encoder on the real canvas, and the sparse encoder on the
// poisoned copy. All three agree pixel for pixel, and the two PNG streams are
// byte identical.
std::vector<uint8_t> require_sparse_preview_matches(const std::vector<uint8_t> &sparse_src,
                                                    const std::vector<uint8_t> &dense_src,
                                                    size_t w, size_t h, size_t n,
                                                    const sla::TileMap &tiles)
{
    INFO("w = " << w << ", h = " << h << ", n = " << n);
    REQUIRE(sparse_src.size() == w * h);
    REQUIRE(dense_src.size() == w * h);

    size_t new_w = 0, new_h = 0;
    preview_size(w, h, n, new_w, new_h);

    // The gates the encoders test, checked here too: a case that would silently
    // fall to another path must fail loudly instead of comparing the wrong code.
    REQUIRE(1.0 / (1.0 / double(n)) == double(n));   // 1/scale is exactly n
    REQUIRE(new_w * n <= w);                         // fixed-block path
    REQUIRE(new_h * n <= h);
    REQUIRE(sla::TileMap::T % n == 0);               // sparse path
    REQUIRE(tiles.width_px() == w);
    REQUIRE(tiles.height_px() == h);

    const sla::EncodedRaster dense_enc  = dense_preview_png(dense_src, w, h, n);
    const sla::EncodedRaster sparse_enc = sparse_preview_png(sparse_src, w, h, n, tiles);

    const std::vector<uint8_t> dense_bytes  = bytes_of(dense_enc);
    const std::vector<uint8_t> sparse_bytes = bytes_of(sparse_enc);
    REQUIRE(sparse_bytes.size() == dense_bytes.size());
    REQUIRE(first_difference(sparse_bytes, dense_bytes) == NoDifference);   // PNG byte identical

    const std::vector<uint8_t> dense_px  = decode_preview(dense_enc, new_w, new_h);
    const std::vector<uint8_t> sparse_px = decode_preview(sparse_enc, new_w, new_h);

    std::vector<uint8_t> expected(new_w * new_h);
    sla::preview_box_downscale_integer_reference(dense_src.data(), w, expected.data(),
                                                 new_w, new_h, 1, n);

    REQUIRE(first_difference(dense_px, expected) == NoDifference);    // the oracle
    REQUIRE(first_difference(sparse_px, expected) == NoDifference);
    return sparse_px;
}

// Buffer plus tile map plus poisoned copy, built by hand so a test controls
// exactly which tiles are written.
struct PreviewCanvas {
    size_t w = 0, h = 0;
    std::vector<uint8_t> px;         // the canvas as it really is
    std::vector<uint8_t> poisoned;   // the same, clean areas filled with poison
    sla::TileMap tiles;
};

// 400 x 240 (10 x 6 tiles) with tile (1, 1) filled white and nothing else, so
// every factor in {4, 5, 8, 10} divides both the tile edge and the tile origin.
PreviewCanvas single_tile_canvas()
{
    PreviewCanvas c;
    c.w = 400;
    c.h = 240;
    c.px.assign(c.w * c.h, 0);
    c.tiles = sla::TileMap(c.w, c.h);
    for (size_t y = 40; y < 80; ++y)
        write_span(c.px, c.tiles, c.w, y, 40, 80, 255);
    c.poisoned = poison_clean_areas(c.px, c.w, c.h, c.tiles);
    return c;
}

// ---------------------------------------------------------------------------
// Tile-unaware consumer helpers (tasks 2.15).
//
// PNGRasterEncoder takes only (ptr, w, h, num_components) and read_pixel reads
// m_buf directly, so neither can consult the tile map. What has to be shown is
// that this still holds in the one place where the map shapes the buffer: after
// a partial reset, where only the written spans were refilled.
// ---------------------------------------------------------------------------

// The layer PNG as SLAArchiveWriter stores it, taken through a raster that does
// carry a tile map.
sla::EncodedRaster layer_png(const sla::RasterBase &raster)
{
    return raster.encode(sla::PNGRasterEncoder{});
}

// The same encoder on a bare buffer: no raster and no tile map in sight.
sla::EncodedRaster layer_png_of_buffer(const std::vector<uint8_t> &px, size_t w, size_t h)
{
    return sla::PNGRasterEncoder{}(px.data(), w, h, 1);
}

std::vector<uint8_t> decode_layer_png(const sla::EncodedRaster &enc, size_t w, size_t h)
{
    png::ImageGreyscale img;
    REQUIRE(png::decode_png({enc.data(), enc.size()}, img));
    REQUIRE(img.cols == w);
    REQUIRE(img.rows == h);
    REQUIRE(img.buf.size() == w * h);
    return img.buf;
}

// read_pixel of two rasters over a coarse grid. A step that shares no factor
// with the tile edge lands the samples at every offset inside a tile, so the
// whole canvas is covered without reading all of it (X3).
void require_read_pixel_agrees_on_grid(const sla::RasterGrayscaleAA &a,
                                       const sla::RasterGrayscaleAA &b,
                                       size_t w, size_t h, size_t step)
{
    TileList differing;
    for (size_t y = 0; y < h; y += step)
        for (size_t x = 0; x < w; x += step)
            if (a.read_pixel(x, y) != b.read_pixel(x, y) && differing.size() < 8)
                differing.emplace_back(x, y);
    REQUIRE(differing.empty());
}

void require_read_pixel_agrees_in_window(const sla::RasterGrayscaleAA &a,
                                         const sla::RasterGrayscaleAA &b,
                                         size_t x0, size_t y0, size_t x1, size_t y1)
{
    TileList differing;
    for (size_t y = y0; y < y1; ++y)
        for (size_t x = x0; x < x1; ++x)
            if (a.read_pixel(x, y) != b.read_pixel(x, y) && differing.size() < 8)
                differing.emplace_back(x, y);
    REQUIRE(differing.empty());
}

// read_pixel and the decoded layer PNG have to describe the same buffer.
void require_read_pixel_matches_png(const sla::RasterGrayscaleAA &raster,
                                    const std::vector<uint8_t> &png_px, size_t w, size_t h)
{
    REQUIRE(png_px.size() == w * h);
    TileList differing;
    for (size_t y = 0; y < h; ++y)
        for (size_t x = 0; x < w; ++x)
            if (raster.read_pixel(x, y) != png_px[y * w + x] && differing.size() < 8)
                differing.emplace_back(x, y);
    REQUIRE(differing.empty());
}

// ---------------------------------------------------------------------------
// Random differential helpers (tasks 2.17).
// ---------------------------------------------------------------------------

// Pseudo-random star-shaped polygons from a fixed seed, so a failure is
// reproducible and a rerun draws exactly the same layer. Each vertex gets its
// own radius, which makes the outline jagged: plenty of slanted edges at
// fractional pixel positions, hence plenty of real anti-aliased gray. Centres
// may sit near an edge and let a shape hang off the canvas, which is wanted --
// the marking has to stay inside the tile grid there.
std::vector<ExPolygon> random_polygons(size_t w, size_t h, unsigned seed, size_t count,
                                       double r_min, double r_max)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> ux(0.0, double(w));
    std::uniform_real_distribution<double> uy(0.0, double(h));
    std::uniform_real_distribution<double> uradius(r_min, r_max);
    std::uniform_real_distribution<double> ujitter(0.45, 1.0);
    std::uniform_real_distribution<double> uphase(0.0, 2. * PI);
    std::uniform_int_distribution<int>     uverts(3, 9);

    std::vector<ExPolygon> out;
    out.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        const double cx = ux(rng), cy = uy(rng);
        const double r = uradius(rng), phase = uphase(rng);
        const int    verts = uverts(rng);

        std::vector<std::pair<double, double>> pts;
        pts.reserve(size_t(verts));
        for (int k = 0; k < verts; ++k) {
            const double a  = phase + 2. * PI * double(k) / double(verts);
            const double rk = r * ujitter(rng);
            pts.emplace_back(cx + rk * std::cos(a), cy + rk * std::sin(a));
        }
        out.push_back(pixel_polygon(h, pts));
    }
    return out;
}

// One random layer through every sparse path: the tile map is checked, the
// clean areas are poisoned, and the layer file and all four previews have to
// come out byte for byte as the dense code makes them. Returns true when this
// layer actually left some tiles clean, which is what makes the sparse paths
// differ from the dense ones at all.
bool require_random_layer_matches(const sla::RasterGrayscaleAA &raster, size_t w, size_t h)
{
    const sla::TileMap *tiles = raster.written_tiles();
    REQUIRE(tiles != nullptr);

    const std::vector<uint8_t> px = raster_pixels(raster, w, h);
    require_marks_cover_pixels(px, w, h, *tiles);        // I2, so poisoning is safe
    require_row_summaries_consistent(*tiles);            // N2

    const size_t total = tiles->tiles_x() * tiles->tiles_y();
    const size_t dirty = marked_tiles(*tiles).size();
    REQUIRE(dirty > 0);                                  // the layer is not empty

    const std::vector<uint8_t> poisoned = poison_clean_areas(px, w, h, *tiles);

    require_sparse_rle_matches(poisoned, px, w, h, *tiles);
    for (size_t n : {size_t(4), size_t(5), size_t(8), size_t(10)}) {
        INFO("n = " << n);
        require_sparse_preview_matches(poisoned, px, w, h, n, *tiles);
    }
    return dirty < total;
}

} // namespace

TEST_CASE("Raster scan fast paths are enabled for this run", "[raster-scan]")
{
    // The differential tests compare the reference against itself when the fast
    // paths are switched off, so say so instead of passing silently.
    if (!sla::raster_fastpath_enabled())
        WARN("SLA_RASTER_FASTPATH=0: [raster-scan] tests do not exercise the fast paths");
    SUCCEED();
}

// --- RLE (design D2) --------------------------------------------------------

TEST_CASE("RLE fast path matches reference on full 16K layers", "[raster-scan]")
{
    const size_t W = 15120, H = 6230, N = W * H;
    std::vector<uint8_t> px(N, 0);

    {
        INFO("all black");
        const auto runs = require_rle_matches_reference(px.data(), W, H);
        REQUIRE(runs.size() == 1);
        REQUIRE(runs[0].value == 0);
        REQUIRE(runs[0].length == N);
    }

    for (uint8_t value : {uint8_t(255), uint8_t(128)}) {
        INFO("single pixel value " << int(value));

        px[0] = value;
        {
            INFO("pixel (0, 0)");
            const auto runs = require_rle_matches_reference(px.data(), W, H);
            REQUIRE(runs.size() == 2);
            REQUIRE(has_run(runs, value, 1));
            REQUIRE(has_run(runs, 0, uint32_t(N - 1)));
        }
        px[0] = 0;

        px[N - 1] = value;
        {
            INFO("pixel (W - 1, H - 1)");
            const auto runs = require_rle_matches_reference(px.data(), W, H);
            REQUIRE(runs.size() == 2);
            REQUIRE(runs[0].value == 0);
            REQUIRE(runs[1].value == value);
            REQUIRE(runs[1].length == 1);
        }
        px[N - 1] = 0;
    }

    std::fill(px.begin(), px.end(), uint8_t(255));
    {
        INFO("all white");
        const auto runs = require_rle_matches_reference(px.data(), W, H);
        REQUIRE(runs.size() == 1);
        REQUIRE(runs[0].value == 255);
        REQUIRE(runs[0].length == N);
    }
}

TEST_CASE("RLE fast path matches reference on gray noise", "[raster-scan]")
{
    std::mt19937 rng(20260915u);
    const size_t W = 1021, H = 37;
    std::vector<uint8_t> px(W * H);

    // Uniform noise: almost every byte starts a new run.
    for (uint8_t &v : px) v = uint8_t(rng() % 256);
    require_rle_matches_reference(px.data(), W, H);

    // AA-like edges: black and white spans broken every 1 to 3 pixels by grays.
    for (size_t i = 0; i < px.size();) {
        const uint32_t kind = rng() % 4;
        const size_t   len  = kind == 0 ? 1 + rng() % 3 : 1 + rng() % 40;
        const uint8_t  v    = kind == 0 ? uint8_t(1 + rng() % 254) : (kind == 1 ? uint8_t(255) : uint8_t(0));
        for (size_t k = 0; k < len && i < px.size(); ++k) px[i++] = v;
    }
    require_rle_matches_reference(px.data(), W, H);

    // Real anti-aliased raster.
    const size_t AW = 1003, AH = 607;
    const std::vector<uint8_t> aa = antialiased_pixels(AW, AH);
    require_rle_matches_reference(aa.data(), AW, AH);
}

TEST_CASE("RLE fast path matches reference on runs crossing rows", "[raster-scan]")
{
    const size_t W = 97, H = 61;               // 5917 bytes, not a multiple of 8
    std::vector<uint8_t> px(W * H, 0);

    auto fill = [&](size_t x0, size_t y0, size_t x1, size_t y1, uint8_t v) {
        for (size_t i = y0 * W + x0; i <= y1 * W + x1; ++i) px[i] = v;
        return uint32_t((y1 * W + x1) - (y0 * W + x0) + 1);
    };
    const uint32_t white = fill(90, 3, 4, 40, 255);
    const uint32_t gray  = fill(50, 41, 60, 45, 77);

    const auto runs = require_rle_matches_reference(px.data(), W, H);
    REQUIRE(has_run(runs, 255, white));
    REQUIRE(has_run(runs, 77, gray));
}

TEST_CASE("RLE fast path matches reference at run length format boundaries", "[raster-scan]")
{
    for (uint32_t length : {15u, 16u, 4095u, 4096u, 1048575u, 1048576u})
        for (uint8_t value : {uint8_t(0), uint8_t(255), uint8_t(77)})
            // Lead-in of 0..8 bytes shifts the run across every word alignment.
            for (size_t lead = 0; lead <= 8; ++lead) {
                INFO("length " << length << ", value " << int(value) << ", lead " << lead);
                const uint8_t other = value == 3 ? 9 : 3;
                std::vector<uint8_t> px;
                px.reserve(lead + length + 3);
                for (size_t k = 0; k < lead; ++k) px.push_back(k % 2 ? other : uint8_t(other + 1));
                if (lead > 0) px.back() = other;
                px.insert(px.end(), length, value);
                px.insert(px.end(), 3, other);

                const auto runs = require_rle_matches_reference(px.data(), px.size(), 1);
                REQUIRE(has_run(runs, value, length));
            }
}

TEST_CASE("RLE fast path matches reference when length is not a multiple of 8", "[raster-scan]")
{
    // 7 x 3 = 21 bytes: 16 black then 5 white. The bytes after the buffer repeat
    // the last value, so a read past the end would lengthen the final run.
    {
        std::vector<uint8_t> storage(64, 255);
        std::fill(storage.begin(), storage.begin() + 16, uint8_t(0));

        const auto runs = require_rle_matches_reference(storage.data(), 7, 3);
        REQUIRE(runs.size() == 2);
        REQUIRE(runs[0].value == 0);
        REQUIRE(runs[0].length == 16);
        REQUIRE(runs[1].value == 255);
        REQUIRE(runs[1].length == 5);
    }

    // Every length from 1 to 64, same trailing-bytes trap.
    std::mt19937 rng(7u);
    for (size_t n = 1; n <= 64; ++n) {
        std::vector<uint8_t> storage(n + 16);
        for (size_t i = 0; i < n; ++i) storage[i] = uint8_t((rng() % 3) * 127);
        std::fill(storage.begin() + n, storage.end(), storage[n - 1]);
        require_rle_matches_reference(storage.data(), n, 1);
    }
}

TEST_CASE("RLE with num_components = 3 uses the reference walk", "[raster-scan]")
{
    // Only every third byte is a sample. The bytes in between differ from it, so
    // an encoder that walked the buffer contiguously would produce other runs.
    std::mt19937 rng(3u);
    const size_t W = 33, H = 7;
    std::vector<uint8_t> px(W * H * 3);
    for (size_t p = 0; p < W * H; ++p) {
        const uint8_t sample = (p / 5) % 2 ? uint8_t(255) : uint8_t(0);
        px[p * 3]     = sample;
        px[p * 3 + 1] = uint8_t(1 + rng() % 254);
        px[p * 3 + 2] = uint8_t(1 + rng() % 254);
    }

    require_rle_matches_reference(px.data(), W, H, 3);
}

// --- Preview downscale (design D3) ------------------------------------------

TEST_CASE("Preview fast path matches reference on fixed-seed random buffers", "[raster-scan]")
{
    // 1003 x 207 leaves a partial block on the right and at the bottom for every N.
    const size_t W = 1003, H = 207;
    std::mt19937 rng(20260915u);

    for (size_t n : {4u, 5u, 8u, 10u}) {
        // Lit probability 0, 1/64, 1/4 and 1; lit pixels are white or gray.
        for (uint32_t lit_in_64 : {0u, 1u, 16u, 64u}) {
            INFO("lit " << lit_in_64 << "/64");
            std::vector<uint8_t> px(W * H, 0);
            for (uint8_t &v : px)
                if (rng() % 64 < lit_in_64)
                    v = rng() % 2 ? uint8_t(255) : uint8_t(1 + rng() % 254);
            require_preview_matches_reference(px, W, H, n);
        }

        std::vector<uint8_t> white(W * H, 255);
        require_preview_matches_reference(white, W, H, n);
    }
}

TEST_CASE("Preview fast path matches reference on an anti-aliased raster", "[raster-scan]")
{
    const size_t W = 1003, H = 607;
    const std::vector<uint8_t> px = antialiased_pixels(W, H);

    size_t grays = 0;
    for (uint8_t v : px) grays += v != 0 && v != 255;
    REQUIRE(grays > 0);

    for (size_t n : {4u, 5u, 8u, 10u})
        require_preview_matches_reference(px, W, H, n);
}

TEST_CASE("Preview fast path matches reference on a 7536 x 3240 canvas at N = 5", "[raster-scan]")
{
    // 7536 = 5 * 1507 + 1: the last column is outside every block and ignored.
    const size_t W = 7536, H = 3240, n = 5;
    std::mt19937 rng(8u);

    std::vector<uint8_t> px(W * H, 0);
    for (uint8_t &v : px)
        if (rng() % 32 == 0) v = rng() % 2 ? uint8_t(255) : uint8_t(1 + rng() % 254);

    // Solid and gray bands along the right and bottom edges, including the
    // ignored column, so blocks at the boundary carry non-zero words.
    for (size_t y = 0; y < H; ++y)
        for (size_t x = W - 23; x < W; ++x) px[y * W + x] = (x + y) % 3 ? uint8_t(255) : uint8_t(90);
    for (size_t y = H - 17; y < H; ++y)
        for (size_t x = 0; x < W; ++x) px[y * W + x] = (x / 7) % 2 ? uint8_t(200) : uint8_t(0);

    require_preview_matches_reference(px, W, H, n);
}


// ---------------------------------------------------------------------------
// Tile marking (design D4/D5, tasks 2.9). These drive a real raster: the tiles
// are marked by TileTrackingPixfmt as agg writes, so what is checked is the
// drawing pipeline, not the map's own bookkeeping.
//
// The checks are "every written pixel is in a marked tile" plus "no marked tile
// sits outside the shape". Exact equality is deliberately not required: marking
// more than was written is allowed by I2, and agg does hand the adapter span
// ends whose coverage is zero.
// ---------------------------------------------------------------------------

TEST_CASE("Tile marking follows simple geometry", "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    const size_t W = 200, H = 130;              // 5 x 4 tiles, the bottom edge partial
    const sla::Resolution res{W, H};
    const sla::PixelDim   pixdim{TileMmPerPx, TileMmPerPx};

    SECTION("single pixel")
    {
        sla::RasterGrayscaleAAGammaPower raster(res, pixdim, {}, 1.);
        raster.draw_binary(pixel_rect(H, 45, 33, 46, 34));

        const sla::TileMap *tiles = raster.written_tiles();
        REQUIRE(tiles != nullptr);
        REQUIRE(tiles->tiles_x() == 5);
        REQUIRE(tiles->tiles_y() == 4);

        const std::vector<uint8_t> px = raster_pixels(raster, W, H);
        REQUIRE(px[33 * W + 45] == 255);
        require_marks_cover_pixels(px, W, H, *tiles);
        require_marks_within(*tiles, W, H, 45, 33, 46, 34);
        REQUIRE(marked_tiles(*tiles) == tile_list({{1, 0}}));
    }

    SECTION("horizontal segment")
    {
        sla::RasterGrayscaleAAGammaPower raster(res, pixdim, {}, 1.);
        raster.draw_binary(pixel_rect(H, 10, 100, 70, 101));

        const sla::TileMap *tiles = raster.written_tiles();
        REQUIRE(tiles != nullptr);

        const std::vector<uint8_t> px = raster_pixels(raster, W, H);
        REQUIRE(px[100 * W + 10] == 255);
        REQUIRE(px[100 * W + 69] == 255);
        require_marks_cover_pixels(px, W, H, *tiles);
        require_marks_within(*tiles, W, H, 10, 100, 70, 101);
        REQUIRE(marked_tiles(*tiles) == tile_list({{0, 2}, {1, 2}}));
        REQUIRE_FALSE(tiles->row_written(0));
        REQUIRE(tiles->row_written(2));
    }

    SECTION("triangle inside one tile")
    {
        sla::RasterGrayscaleAAGammaPower raster(res, pixdim, {}, 1.);
        raster.draw_binary(pixel_polygon(H, {{170, 10}, {190, 10}, {180, 30}}));

        const sla::TileMap *tiles = raster.written_tiles();
        REQUIRE(tiles != nullptr);

        const std::vector<uint8_t> px = raster_pixels(raster, W, H);
        REQUIRE(px[11 * W + 180] == 255);
        require_marks_cover_pixels(px, W, H, *tiles);
        require_marks_within(*tiles, W, H, 170, 10, 190, 30);
        REQUIRE(marked_tiles(*tiles) == tile_list({{4, 0}}));
    }
}

TEST_CASE("Tile marking at tile boundaries and canvas corners", "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    SECTION("crossing the x = 80 tile boundary")
    {
        const size_t W = 200, H = 130;
        sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
        raster.draw_binary(pixel_rect(H, 78, 10, 83, 12));

        const sla::TileMap *tiles = raster.written_tiles();
        REQUIRE(tiles != nullptr);

        const std::vector<uint8_t> px = raster_pixels(raster, W, H);
        REQUIRE(px[10 * W + 79] == 255);         // last pixel of tile 1
        REQUIRE(px[10 * W + 80] == 255);         // first pixel of tile 2
        require_marks_cover_pixels(px, W, H, *tiles);
        require_marks_within(*tiles, W, H, 78, 10, 83, 12);
        REQUIRE(marked_tiles(*tiles) ==
                tile_list({{1, 0}, {2, 0}}));
    }

    SECTION("crossing the y = 80 tile boundary")
    {
        const size_t W = 200, H = 130;
        sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
        raster.draw_binary(pixel_rect(H, 10, 78, 12, 83));

        const sla::TileMap *tiles = raster.written_tiles();
        REQUIRE(tiles != nullptr);

        const std::vector<uint8_t> px = raster_pixels(raster, W, H);
        REQUIRE(px[79 * W + 10] == 255);
        REQUIRE(px[80 * W + 10] == 255);
        require_marks_cover_pixels(px, W, H, *tiles);
        require_marks_within(*tiles, W, H, 10, 78, 12, 83);
        REQUIRE(marked_tiles(*tiles) ==
                tile_list({{0, 1}, {0, 2}}));
        REQUIRE_FALSE(tiles->row_written(0));
        REQUIRE(tiles->row_written(1));
        REQUIRE(tiles->row_written(2));
    }

    SECTION("origin and far corner of a 16K canvas")
    {
        const size_t W = 15120, H = 6230;        // 378 x 156 tiles, bottom edge partial

        {
            sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
            raster.draw_binary(pixel_rect(H, 0, 0, 1, 1));

            const sla::TileMap *tiles = raster.written_tiles();
            REQUIRE(tiles != nullptr);
            REQUIRE(tiles->tiles_x() == 378);
            REQUIRE(tiles->tiles_y() == 156);

            const std::vector<uint8_t> px = raster_pixels(raster, W, H);
            REQUIRE(px[0] == 255);
            require_marks_cover_pixels(px, W, H, *tiles);
            require_marks_within(*tiles, W, H, 0, 0, 1, 1);
            REQUIRE(marked_tiles(*tiles) == tile_list({{0, 0}}));
        }

        {
            sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
            raster.draw_binary(pixel_rect(H, W - 1, H - 1, W, H));

            const sla::TileMap *tiles = raster.written_tiles();
            REQUIRE(tiles != nullptr);

            const std::vector<uint8_t> px = raster_pixels(raster, W, H);
            REQUIRE(px[(H - 1) * W + (W - 1)] == 255);
            require_marks_cover_pixels(px, W, H, *tiles);
            require_marks_within(*tiles, W, H, W - 1, H - 1, W, H);
            REQUIRE(marked_tiles(*tiles) == tile_list({{377, 155}}));

            // 15120 = 378 * 40 exactly, so only the bottom edge is clipped
            // here: 6230 = 155 * 40 + 30 (design D7).
            const sla::TileMap::PixelSpan cols = tiles->pixel_columns(377, 378);
            const sla::TileMap::PixelSpan rows = tiles->pixel_rows(155);
            REQUIRE(cols.begin == 15080);
            REQUIRE(cols.end == 15120);
            REQUIRE(rows.begin == 6200);
            REQUIRE(rows.end == 6230);
        }
    }
}

TEST_CASE("Anti-aliased edges mark their tiles and row summaries", "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    const size_t W = 400, H = 240;               // 10 x 6 tiles
    sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);

    // A slanted band: its long edges fall between pixel centres, so the border
    // pixels come out gray rather than 0 or 255.
    raster.draw(pixel_polygon(H, {{30.5, 20.25}, {360.75, 200.5}, {350.5, 215.75}, {20.25, 35.5}}));

    const sla::TileMap *tiles = raster.written_tiles();
    REQUIRE(tiles != nullptr);

    const std::vector<uint8_t> px = raster_pixels(raster, W, H);
    size_t gray = 0, solid = 0;
    for (uint8_t v : px) {
        if (v == 0) continue;
        if (v == 255) ++solid; else ++gray;
    }
    REQUIRE(solid > 0);
    REQUIRE(gray > 0);                           // anti-aliased edge really is there

    require_marks_cover_pixels(px, W, H, *tiles);
    require_row_summaries_consistent(*tiles);
    require_marks_within(*tiles, W, H, 20, 20, 361, 216);
    REQUIRE(marked_tiles(*tiles).size() > 1);
}

TEST_CASE("reset clears every tile and row summary", "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    const size_t W = 400, H = 240;
    sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
    raster.draw(pixel_polygon(H, {{30.5, 20.25}, {360.75, 200.5}, {350.5, 215.75}, {20.25, 35.5}}));

    const sla::TileMap *tiles = raster.written_tiles();
    REQUIRE(tiles != nullptr);
    REQUIRE(marked_tiles(*tiles).size() > 1);

    raster.reset();

    REQUIRE(tiles->tiles_x() == 10);
    REQUIRE(tiles->tiles_y() == 6);
    REQUIRE(marked_tiles(*tiles).empty());
    for (size_t ty = 0; ty < tiles->tiles_y(); ++ty)
        REQUIRE_FALSE(tiles->row_written(ty));

    // The map still works after the reset: one pixel marks one tile again.
    raster.draw_binary(pixel_rect(H, 200, 120, 201, 121));
    const std::vector<uint8_t> px = raster_pixels(raster, W, H);
    REQUIRE(px[120 * W + 200] == 255);
    require_marks_cover_pixels(px, W, H, *tiles);
    require_marks_within(*tiles, W, H, 200, 120, 201, 121);
    REQUIRE(marked_tiles(*tiles) == tile_list({{5, 3}}));
}


TEST_CASE("Tile marking on canvases that are not a whole number of tiles",
          "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    SECTION("8K landscape 7536 x 3240: right edge 16 px, bottom edge exact")
    {
        const size_t W = 7536, H = 3240;         // 189 x 81 tiles
        sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
        raster.draw_binary(pixel_rect(H, W - 1, H - 1, W, H));

        const sla::TileMap *tiles = raster.written_tiles();
        REQUIRE(tiles != nullptr);
        REQUIRE(tiles->tiles_x() == 189);
        REQUIRE(tiles->tiles_y() == 81);

        REQUIRE(raster.read_pixel(W - 1, H - 1) == 255);
        require_window_marks(raster, *tiles, W - 3, H - 3, W, H);
        REQUIRE(marked_tiles(*tiles) == tile_list({{188, 80}}));

        // 7536 = 188 * 40 + 16, so the right edge tile is 16 px wide, while
        // 3240 = 81 * 40 comes out exact and the bottom row is a full tile.
        require_tile_span(*tiles, 188, 80, 7520, 7536, 3200, 3240);
        require_tile_span(*tiles, 187, 79, 7480, 7520, 3160, 3200);   // a full tile
    }

    SECTION("79 x 81: right tile column 39 px wide, bottom tile row 1 px tall")
    {
        const size_t W = 79, H = 81;             // 2 x 3 tiles
        sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
        raster.draw_binary(pixel_rect(H, 78, 80, 79, 81));

        const sla::TileMap *tiles = raster.written_tiles();
        REQUIRE(tiles != nullptr);
        REQUIRE(tiles->tiles_x() == 2);
        REQUIRE(tiles->tiles_y() == 3);

        REQUIRE(raster.read_pixel(78, 80) == 255);
        require_window_marks(raster, *tiles, 0, 0, W, H);
        REQUIRE(marked_tiles(*tiles) == tile_list({{1, 2}}));
        REQUIRE_FALSE(tiles->row_written(0));
        REQUIRE_FALSE(tiles->row_written(1));
        REQUIRE(tiles->row_written(2));

        require_tile_span(*tiles, 0, 0, 0, 40, 0, 40);    // a full tile
        require_tile_span(*tiles, 1, 2, 40, 79, 80, 81);  // 39 px wide, 1 px tall
    }

    SECTION("81 x 1: right tile column 1 px wide, single tile row 1 px tall")
    {
        const size_t W = 81, H = 1;              // 3 x 1 tiles
        sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
        raster.draw_binary(pixel_rect(H, 80, 0, 81, 1));

        const sla::TileMap *tiles = raster.written_tiles();
        REQUIRE(tiles != nullptr);
        REQUIRE(tiles->tiles_x() == 3);
        REQUIRE(tiles->tiles_y() == 1);

        REQUIRE(raster.read_pixel(80, 0) == 255);
        require_window_marks(raster, *tiles, 0, 0, W, H);
        REQUIRE(marked_tiles(*tiles) == tile_list({{2, 0}}));

        require_tile_span(*tiles, 0, 0, 0, 40, 0, 1);
        require_tile_span(*tiles, 2, 0, 80, 81, 0, 1);   // one pixel wide
    }
}

TEST_CASE("Tile marking on a portrait 16K canvas", "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    // Portrait swaps the axes: 6230 x 15120 is 156 x 378 tiles, so the partial
    // edge moves to the right (6230 = 155 * 40 + 30) while the bottom comes out
    // exact (15120 = 378 * 40). Only small windows are read here; a full scan
    // would be 94 million pixels.
    const size_t W = 6230, H = 15120;

    SECTION("right edge of a partial tile column")
    {
        sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
        raster.draw_binary(pixel_rect(H, W - 1, 0, W, 1));

        const sla::TileMap *tiles = raster.written_tiles();
        REQUIRE(tiles != nullptr);
        REQUIRE(tiles->tiles_x() == 156);
        REQUIRE(tiles->tiles_y() == 378);

        REQUIRE(raster.read_pixel(W - 1, 0) == 255);
        require_window_marks(raster, *tiles, W - 3, 0, W, 3);
        REQUIRE(marked_tiles(*tiles) == tile_list({{155, 0}}));

        require_tile_span(*tiles, 155, 0, 6200, 6230, 0, 40);     // 30 px wide
        require_tile_span(*tiles, 155, 377, 6200, 6230, 15080, 15120);
    }

    SECTION("bottom row and the tile boundary at x = 6160")
    {
        sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
        raster.draw_binary(pixel_rect(H, 0, H - 1, 1, H));            // pixel (0, 15119)
        raster.draw_binary(pixel_rect(H, 6155, 100, 6165, 102));      // crosses x = 6160

        const sla::TileMap *tiles = raster.written_tiles();
        REQUIRE(tiles != nullptr);

        REQUIRE(raster.read_pixel(0, H - 1) == 255);
        REQUIRE(raster.read_pixel(6159, 100) == 255);                 // last px of tile 153
        REQUIRE(raster.read_pixel(6160, 100) == 255);                 // first px of tile 154
        require_window_marks(raster, *tiles, 0, H - 3, 3, H);
        require_window_marks(raster, *tiles, 6153, 98, 6167, 104);
        REQUIRE(marked_tiles(*tiles) == tile_list({{153, 2}, {154, 2}, {0, 377}}));

        require_tile_span(*tiles, 153, 2, 6120, 6160, 80, 120);       // a full tile
        require_tile_span(*tiles, 0, 377, 0, 40, 15080, 15120);       // exact bottom
    }
}


// ---------------------------------------------------------------------------
// Partial reset (design D6 (1), tasks 2.11). reset() only refills the written
// spans, so what has to be shown is that the result is what a full clear would
// leave behind -- buffer and tile map alike -- layer after layer.
// ---------------------------------------------------------------------------

TEST_CASE("reset restores the buffer a full clear would leave", "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    const size_t W = 400, H = 240;
    sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
    draw_scattered(raster, H);

    const std::vector<uint8_t> drawn = raster_pixels(raster, W, H);
    size_t lit = 0;
    for (uint8_t v : drawn)
        if (v != 0) ++lit;
    REQUIRE(lit > 0);                                   // the layer is not empty
    REQUIRE(drawn[239 * W + 399] == 255);               // corner really was written

    raster.reset();

    // Byte for byte against a raster that was never drawn on at all.
    sla::RasterGrayscaleAAGammaPower fresh({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
    const std::vector<uint8_t> after = raster_pixels(raster, W, H);
    const std::vector<uint8_t> blank = raster_pixels(fresh, W, H);
    REQUIRE(after.size() == blank.size());
    REQUIRE(first_difference(after, blank) == NoDifference);

    size_t survivors = 0;
    for (uint8_t v : after)
        if (v != 0) ++survivors;
    REQUIRE(survivors == 0);
}

TEST_CASE("reset zeroes the tile map and keeps its shape", "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    // 8K: two opposite corner tiles only, so the 16 px wide right edge tile is
    // one of the ones being cleared. Verified through small windows, not a scan
    // of all 24 million pixels.
    const size_t W = 7536, H = 3240;
    sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
    raster.draw_binary(pixel_rect(H, 0, 0, 2, 2));
    raster.draw_binary(pixel_rect(H, W - 2, H - 2, W, H));

    const sla::TileMap *tiles = raster.written_tiles();
    REQUIRE(tiles != nullptr);
    REQUIRE(marked_tiles(*tiles) == tile_list({{0, 0}, {188, 80}}));
    REQUIRE(raster.read_pixel(0, 0) == 255);
    REQUIRE(raster.read_pixel(W - 1, H - 1) == 255);

    raster.reset();

    REQUIRE(tiles->tiles_x() == 189);                   // shape survives the reset
    REQUIRE(tiles->tiles_y() == 81);
    REQUIRE(tiles->width_px() == W);
    REQUIRE(tiles->height_px() == H);
    REQUIRE(marked_tiles(*tiles).empty());
    for (size_t ty = 0; ty < tiles->tiles_y(); ++ty)
        REQUIRE_FALSE(tiles->row_written(ty));

    for (size_t y = 0; y < 3; ++y)
        for (size_t x = 0; x < 3; ++x)
            REQUIRE(raster.read_pixel(x, y) == 0);
    for (size_t y = H - 3; y < H; ++y)
        for (size_t x = W - 3; x < W; ++x)
            REQUIRE(raster.read_pixel(x, y) == 0);
}

TEST_CASE("A reused raster gives the same layers as fresh ones", "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    const size_t W = 400, H = 240;

    // The sequence from the task: full plate, empty layer, a single pixel in the
    // far corner, then a large area. Each one follows a reset of the previous.
    const std::vector<LayerDraw> layers = {
        [&](sla::RasterGrayscaleAA &r) { r.draw_binary(pixel_rect(H, 0, 0, W, H)); },
        [](sla::RasterGrayscaleAA &) {},
        [&](sla::RasterGrayscaleAA &r) { r.draw_binary(pixel_rect(H, W - 1, H - 1, W, H)); },
        [&](sla::RasterGrayscaleAA &r) { draw_scattered(r, H); },
    };

    sla::RasterGrayscaleAAGammaPower reused({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);

    for (size_t i = 0; i < layers.size(); ++i) {
        INFO("layer " << i);

        reused.reset();                                 // what draw_layers() does
        layers[i](reused);

        sla::RasterGrayscaleAAGammaPower fresh({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
        layers[i](fresh);

        const std::vector<uint8_t> a = raster_pixels(reused, W, H);
        const std::vector<uint8_t> b = raster_pixels(fresh, W, H);
        REQUIRE(a.size() == b.size());
        REQUIRE(first_difference(a, b) == NoDifference);

        const sla::TileMap *ta = reused.written_tiles();
        const sla::TileMap *tb = fresh.written_tiles();
        REQUIRE(ta != nullptr);
        REQUIRE(tb != nullptr);
        REQUIRE(marked_tiles(*ta) == marked_tiles(*tb));
        require_marks_cover_pixels(a, W, H, *ta);
        require_row_summaries_consistent(*ta);
    }

    // The empty layer in the middle really did come out empty, so the full plate
    // before it left nothing behind.
    reused.reset();
    const std::vector<uint8_t> after = raster_pixels(reused, W, H);
    size_t survivors = 0;
    for (uint8_t v : after)
        if (v != 0) ++survivors;
    REQUIRE(survivors == 0);
    REQUIRE(marked_tiles(*reused.written_tiles()).empty());
}


// ---------------------------------------------------------------------------
// Post-process flag (design D6 (2), tasks 2.12). The injected post-process
// records its calls, so these tests can see which path ran: one call per row
// segment (sparse) or a single whole-buffer call followed by mark_all().
// ---------------------------------------------------------------------------

TEST_CASE("A zero-preserving pixel-local post-process runs over the written tiles only",
          "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    const size_t W = 400, H = 240;                       // 10 x 6 tiles
    auto log = std::make_shared<PostProcLog>();
    sla::RasterPostProcess post = make_postproc(log, true,
        [](uint8_t *p, size_t w, size_t h) { map_nonzero_to(p, w, h, 100); });

    sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.,
                                            std::move(post));
    raster.draw_binary(pixel_rect(H, 10, 10, 30, 30));       // tile (0, 0)
    raster.draw_binary(pixel_rect(H, 390, 230, 400, 240));   // tile (9, 5), a corner

    const sla::TileMap *tiles = raster.written_tiles();
    REQUIRE(tiles != nullptr);
    REQUIRE(marked_tiles(*tiles) == tile_list({{0, 0}, {9, 5}}));
    const size_t expected_pixels = written_span_pixels(*tiles);
    REQUIRE(expected_pixels == 3200);                    // two whole 40 x 40 tiles

    raster.apply_postprocess();

    // The map is untouched: mark_all() was not called, so the clean tiles stay
    // clean and the next reset() is still a partial one.
    REQUIRE(marked_tiles(*tiles) == tile_list({{0, 0}, {9, 5}}));

    // One call per pixel row of a written span, and nothing outside them.
    REQUIRE(log->calls.size() == 80);                    // 40 rows in each of two tile rows
    REQUIRE(log->pixels_seen == expected_pixels);
    std::vector<size_t> wrong_shape;
    for (size_t i = 0; i < log->calls.size(); ++i)
        if ((log->calls[i].h != 1 || log->calls[i].w != 40 || log->calls[i].nc != 1) &&
            wrong_shape.size() < 8)
            wrong_shape.push_back(i);
    REQUIRE(wrong_shape.empty());

    // The mapping really was applied, and only inside the written tiles.
    REQUIRE(raster.read_pixel(15, 15) == 100);
    REQUIRE(raster.read_pixel(395, 235) == 100);
    REQUIRE(raster.read_pixel(200, 120) == 0);           // clean tile, untouched

    raster.reset();

    sla::RasterGrayscaleAAGammaPower fresh({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
    const std::vector<uint8_t> after = raster_pixels(raster, W, H);
    const std::vector<uint8_t> blank = raster_pixels(fresh, W, H);
    REQUIRE(after.size() == blank.size());
    REQUIRE(first_difference(after, blank) == NoDifference);
    REQUIRE(marked_tiles(*tiles).empty());
}

TEST_CASE("A post-process that may spread runs whole-buffer and marks every tile",
          "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    const size_t W = 400, H = 240;
    auto log = std::make_shared<PostProcLog>();
    // Flag false: like blur > 0, this one also writes a pixel far from anything
    // that was drawn, which is exactly what the whole-buffer path must cover.
    sla::RasterPostProcess post = make_postproc(log, false,
        [](uint8_t *p, size_t w, size_t h) {
            map_nonzero_to(p, w, h, 100);
            p[0] = 50;                                   // spills into a clean tile
        });

    sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.,
                                            std::move(post));
    raster.draw_binary(pixel_rect(H, 150, 90, 260, 140));

    const sla::TileMap *tiles = raster.written_tiles();
    REQUIRE(tiles != nullptr);
    const size_t drawn_tiles = marked_tiles(*tiles).size();
    REQUIRE(drawn_tiles > 0);
    REQUIRE(drawn_tiles < tiles->tiles_x() * tiles->tiles_y());

    raster.apply_postprocess();

    // A single whole-buffer call ...
    REQUIRE(log->calls.size() == 1);
    REQUIRE(log->calls[0].w == W);
    REQUIRE(log->calls[0].h == H);
    REQUIRE(log->pixels_seen == W * H);

    // ... and mark_all(): every tile and every row summary is dirty afterwards.
    REQUIRE(marked_tiles(*tiles).size() == tiles->tiles_x() * tiles->tiles_y());
    for (size_t ty = 0; ty < tiles->tiles_y(); ++ty)
        REQUIRE(tiles->row_written(ty));

    // The spilled pixel sits in a tile nothing was drawn in; it is covered by
    // the map, so require_marks_cover_pixels holds and reset() wipes it.
    REQUIRE(raster.read_pixel(0, 0) == 50);
    const std::vector<uint8_t> processed = raster_pixels(raster, W, H);
    require_marks_cover_pixels(processed, W, H, *tiles);

    raster.reset();

    sla::RasterGrayscaleAAGammaPower fresh({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
    const std::vector<uint8_t> after = raster_pixels(raster, W, H);
    const std::vector<uint8_t> blank = raster_pixels(fresh, W, H);
    REQUIRE(after.size() == blank.size());
    REQUIRE(first_difference(after, blank) == NoDifference);   // no residue
    REQUIRE(raster.read_pixel(0, 0) == 0);
    REQUIRE(marked_tiles(*tiles).empty());
}

TEST_CASE("A non-zero background falls back to the whole-buffer post-process",
          "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    const size_t W = 400, H = 240;
    auto log = std::make_shared<PostProcLog>();
    // The flag says pixel-local and zero-preserving, but the background here is
    // white: skipping the clean tiles would leave them unprocessed, so the
    // raster must ignore the flag.
    sla::RasterPostProcess post = make_postproc(log, true,
        [](uint8_t *p, size_t w, size_t h) {
            for (size_t i = 0; i < w * h; ++i)
                if (p[i] == 0) p[i] = 60;
        });

    sla::_RasterGrayscaleAA raster({W, H}, {TileMmPerPx, TileMmPerPx}, {},
                                   agg::gray8{0}, agg::gray8{255}, agg::gamma_power(1.),
                                   std::move(post));
    raster.draw_binary(pixel_rect(H, 10, 10, 30, 30));       // black on white

    const sla::TileMap *tiles = raster.written_tiles();
    REQUIRE(tiles != nullptr);
    REQUIRE(marked_tiles(*tiles) == tile_list({{0, 0}}));

    raster.apply_postprocess();

    REQUIRE(log->calls.size() == 1);                     // whole buffer, not per row
    REQUIRE(log->calls[0].w == W);
    REQUIRE(log->calls[0].h == H);
    REQUIRE(marked_tiles(*tiles).size() == tiles->tiles_x() * tiles->tiles_y());
    for (size_t ty = 0; ty < tiles->tiles_y(); ++ty)
        REQUIRE(tiles->row_written(ty));

    raster.reset();

    // Back to the white background it was constructed with, everywhere.
    const std::vector<uint8_t> after = raster_pixels(raster, W, H);
    size_t not_white = 0;
    for (uint8_t v : after)
        if (v != 255) ++not_white;
    REQUIRE(not_white == 0);
    REQUIRE(marked_tiles(*tiles).empty());
}


// ---------------------------------------------------------------------------
// Sparse RLE encoding (design D6 (3), tasks 2.13). The sparse encoder must be
// byte-for-byte identical to the dense one, and it must reach that result
// without touching the clean parts of the buffer. Every test below therefore
// hands it a copy whose clean areas hold CleanAreaPoison: if a single one of
// those bytes were read, the runs would change and the comparison would fail.
// ---------------------------------------------------------------------------

TEST_CASE("Sparse RLE matches the dense encoder byte for byte", "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    const size_t W = 400, H = 240;                       // 10 x 6 tiles
    sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);

    // Polygons with anti-aliased slanted edges, a block crossing two tile
    // columns, a block sitting on a tile corner, and several separate patches.
    raster.draw(pixel_polygon(H, {{18.5, 12.25}, {132.75, 40.5}, {60.25, 118.5}}));
    raster.draw_binary(pixel_rect(H, 150, 70, 260, 130));        // crosses x = 160 and 240
    raster.draw_binary(pixel_rect(H, 78, 158, 84, 164));         // sits on a tile corner
    raster.draw_binary(pixel_rect(H, 0, 0, 3, 3));
    raster.draw_binary(pixel_rect(H, 397, 237, 400, 240));
    raster.draw(pixel_polygon(H, {{300.5, 190.25}, {340.75, 200.5}, {310.25, 225.5}}));

    const sla::TileMap *tiles = raster.written_tiles();
    REQUIRE(tiles != nullptr);

    const std::vector<uint8_t> px = raster_pixels(raster, W, H);
    require_marks_cover_pixels(px, W, H, *tiles);        // I2 holds, so poisoning is safe
    require_row_summaries_consistent(*tiles);

    const size_t dirty = marked_tiles(*tiles).size();
    REQUIRE(dirty > 0);
    REQUIRE(dirty < tiles->tiles_x() * tiles->tiles_y());    // clean tiles do exist

    const std::vector<uint8_t> poisoned = poison_clean_areas(px, W, H, *tiles);
    const std::vector<Run> runs = require_sparse_rle_matches(poisoned, px, W, H, *tiles);

    // The pattern really did produce gray (anti-aliased), white and black runs,
    // so the equality above is not the equality of two trivial outputs.
    size_t gray = 0, white = 0, black = 0;
    for (const Run &r : runs) {
        if      (r.value == 0)   ++black;
        else if (r.value == 255) ++white;
        else                     ++gray;
    }
    REQUIRE(gray > 0);
    REQUIRE(white > 0);
    REQUIRE(black > 10);
}

TEST_CASE("Sparse RLE counts clean areas instead of scanning them", "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    SECTION("an empty 8K layer is a single black run")
    {
        const size_t W = 7536, H = 3240;                 // 189 x 81 tiles
        const std::vector<uint8_t> px(W * H, 0);
        sla::TileMap tiles(W, H);                        // nothing written at all
        REQUIRE(tiles.tiles_x() == 189);
        REQUIRE(tiles.tiles_y() == 81);
        REQUIRE(marked_tiles(tiles).empty());

        // Every single pixel is poison here, so the encoder may not read one.
        const std::vector<uint8_t> poisoned(W * H, CleanAreaPoison);
        const std::vector<Run> runs = require_sparse_rle_matches(poisoned, px, W, H, tiles);

        REQUIRE(runs.size() == 1);
        REQUIRE(runs[0].value == 0);
        REQUIRE(runs[0].length == uint32_t(W * H));      // 24,416,640
    }

    SECTION("one lit pixel in the far corner of an 8K layer")
    {
        const size_t W = 7536, H = 3240;
        sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
        raster.draw_binary(pixel_rect(H, W - 1, H - 1, W, H));

        const sla::TileMap *tiles = raster.written_tiles();
        REQUIRE(tiles != nullptr);
        REQUIRE(marked_tiles(*tiles) == tile_list({{188, 80}}));  // the partial corner tile

        const std::vector<uint8_t> px = raster_pixels(raster, W, H);
        REQUIRE(px[(H - 1) * W + (W - 1)] == 255);

        const std::vector<uint8_t> poisoned = poison_clean_areas(px, W, H, *tiles);
        const std::vector<Run> runs = require_sparse_rle_matches(poisoned, px, W, H, *tiles);

        REQUIRE(runs.size() == 2);
        REQUIRE(runs[0].value == 0);
        REQUIRE(runs[0].length == uint32_t(W * H - 1));  // 40 clean tile rows, then row heads
        REQUIRE(runs[1].value == 255);
        REQUIRE(runs[1].length == 1);
    }

    SECTION("a tile marked but left all black still gives one 2,419,200 run")
    {
        // The 16K width from the task. Tile (0, 0) is marked yet holds nothing
        // but background -- over-marking is allowed by I2 -- tile rows 1 to 3
        // are clean, and the first pixel of row 160 is white, so the leading
        // run is exactly 160 rows of 15120.
        const size_t W = 15120, H = 240;                 // 378 x 6 tiles
        std::vector<uint8_t> px(W * H, 0);
        sla::TileMap tiles(W, H);

        tiles.mark_span(0, 0, 40);                       // tile (0, 0): marked, all black
        px[160 * W] = 255;
        tiles.mark_span(0, 160, 1);                      // tile (0, 4): the white pixel

        REQUIRE(tiles.tiles_x() == 378);
        REQUIRE(tiles.tiles_y() == 6);
        REQUIRE(marked_tiles(tiles) == tile_list({{0, 0}, {0, 4}}));
        REQUIRE(tiles.row_written(0));
        for (size_t ty = 1; ty < 4; ++ty)
            REQUIRE_FALSE(tiles.row_written(ty));        // three tile rows skipped whole
        REQUIRE(tiles.row_written(4));

        const std::vector<uint8_t> poisoned = poison_clean_areas(px, W, H, tiles);
        const std::vector<Run> runs = require_sparse_rle_matches(poisoned, px, W, H, tiles);

        REQUIRE(runs.size() == 3);
        REQUIRE(runs[0].value == 0);
        REQUIRE(runs[0].length == 2419200);              // 160 * 15120
        REQUIRE(runs[1].value == 255);
        REQUIRE(runs[1].length == 1);
        REQUIRE(runs[2].value == 0);
        REQUIRE(runs[2].length == uint32_t(W * H) - 2419201);
    }
}

TEST_CASE("Sparse RLE merges background across spans and row ends", "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    // 10 x 4 tiles. In every one of the first 80 pixel rows: white on
    // [80, 130), black on [130, 160) inside the same written span, and a second
    // written span on [240, 320) that holds nothing but black. The columns
    // [0, 80), [160, 240) and [320, 400) are clean, and so is the whole of tile
    // rows 2 and 3.
    const size_t W = 400, H = 160;
    std::vector<uint8_t> px(W * H, 0);
    sla::TileMap tiles(W, H);

    for (size_t y = 0; y < 80; ++y) {
        write_span(px, tiles, W, y, 80, 130, 255);
        write_span(px, tiles, W, y, 130, 160, 0);        // 30 black ending the written span
        write_span(px, tiles, W, y, 240, 320, 0);        // a marked span with no ink in it
    }

    REQUIRE(marked_tiles(tiles) ==
            tile_list({{2, 0}, {3, 0}, {6, 0}, {7, 0},
                       {2, 1}, {3, 1}, {6, 1}, {7, 1}}));
    REQUIRE(tiles.row_written(0));
    REQUIRE(tiles.row_written(1));
    REQUIRE_FALSE(tiles.row_written(2));
    REQUIRE_FALSE(tiles.row_written(3));

    const std::vector<uint8_t> poisoned = poison_clean_areas(px, W, H, tiles);
    const std::vector<Run> runs = require_sparse_rle_matches(poisoned, px, W, H, tiles);

    // Between two white runs the background has to survive five joins: the 30
    // black pixels ending the first written span, the clean columns [160, 240),
    // the marked but black span [240, 320), the clean columns [320, 400), and
    // the 80 pixels opening the next row. One run of 350, not five runs.
    REQUIRE(runs.size() == 161);                         // 81 black + 80 white
    REQUIRE(runs[0].value == 0);
    REQUIRE(runs[0].length == 80);                       // head of row 0
    for (size_t i = 1; i + 1 < runs.size(); ++i) {
        INFO("run " << i);
        if (i % 2 == 1) {
            REQUIRE(runs[i].value == 255);
            REQUIRE(runs[i].length == 50);
        } else {
            REQUIRE(runs[i].value == 0);
            REQUIRE(runs[i].length == 350);              // (400 - 130) + 80
        }
    }

    // The last run runs off the end of row 79 and swallows the clean tile row.
    REQUIRE(runs.back().value == 0);
    REQUIRE(runs.back().length == (W - 130) + 80 * W);           // 32,270

    // No truncation at a span end, and no extra flush where a tile row ends:
    // the two clean tile rows are 40 pixel rows each.
    REQUIRE(count_runs(runs, 0, 30) == 0);
    REQUIRE(count_runs(runs, 0, 40 * W) == 0);
    REQUIRE(count_runs(runs, 0, 80 * W) == 0);

    // And nothing that was poisoned ever reached the output.
    size_t poison_runs = 0;
    for (const Run &r : runs)
        if (r.value == CleanAreaPoison) ++poison_runs;
    REQUIRE(poison_runs == 0);
}


// ---------------------------------------------------------------------------
// Sparse preview downscale (design D6 (4), tasks 2.14). The sparse downscale
// leaves a destination pixel at the 0 it was initialized to when its source
// block sits in a clean tile, so it never reads that block. The poison from
// task 2.13 makes that observable: a block that is read comes out as 0xAA, not
// as 0.
// ---------------------------------------------------------------------------

TEST_CASE("Sparse preview matches the dense downscale for every allowed factor",
          "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    const size_t W = 400, H = 240;                       // 10 x 6 tiles
    sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);

    // Anti-aliased slanted edges, a rectangle crossing two tile columns, and
    // two separate patches well away from the rest.
    raster.draw(pixel_polygon(H, {{18.5, 12.25}, {132.75, 40.5}, {60.25, 118.5}}));
    raster.draw_binary(pixel_rect(H, 150, 90, 260, 140));        // crosses x = 160 and 240
    raster.draw_binary(pixel_rect(H, 0, 0, 6, 6));
    raster.draw(pixel_polygon(H, {{300.5, 190.25}, {340.75, 200.5}, {310.25, 225.5}}));

    const sla::TileMap *tiles = raster.written_tiles();
    REQUIRE(tiles != nullptr);

    const std::vector<uint8_t> px = raster_pixels(raster, W, H);
    require_marks_cover_pixels(px, W, H, *tiles);        // I2 holds, so poisoning is safe
    require_row_summaries_consistent(*tiles);

    const size_t dirty = marked_tiles(*tiles).size();
    REQUIRE(dirty > 0);
    REQUIRE(dirty < tiles->tiles_x() * tiles->tiles_y());    // clean tiles do exist

    const std::vector<uint8_t> poisoned = poison_clean_areas(px, W, H, *tiles);

    // 40 % n == 0 for each of these, so no block can straddle two tiles.
    for (size_t n : {size_t(4), size_t(5), size_t(8), size_t(10)}) {
        INFO("n = " << n);
        const std::vector<uint8_t> prev =
            require_sparse_preview_matches(poisoned, px, W, H, n, *tiles);

        // The preview is neither blank nor solid, so the equality above is not
        // the equality of two trivial buffers.
        size_t lit = 0, gray = 0;
        for (uint8_t v : prev) {
            if (v != 0) ++lit;
            if (v != 0 && v != 255) ++gray;
        }
        REQUIRE(lit > 0);
        REQUIRE(lit < prev.size());
        REQUIRE(gray > 0);                               // averaged edges survive
    }
}

TEST_CASE("Sparse preview leaves clean tiles at zero without reading them",
          "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    // One written tile, everything else poisoned. A destination pixel whose
    // block were read would average 0xAA and come out 170, never 0.
    const PreviewCanvas c = single_tile_canvas();
    REQUIRE(marked_tiles(c.tiles) == tile_list({{1, 1}}));

    for (size_t n : {size_t(4), size_t(5), size_t(8), size_t(10)}) {
        INFO("n = " << n);
        size_t new_w = 0, new_h = 0;
        preview_size(c.w, c.h, n, new_w, new_h);

        const std::vector<uint8_t> prev =
            decode_preview(sparse_preview_png(c.poisoned, c.w, c.h, n, c.tiles), new_w, new_h);

        // Tile (1, 1) covers source pixels [40, 80) x [40, 80), which is
        // exactly blocks [40 / n, 80 / n) on both axes.
        const size_t dx0 = 40 / n, dx1 = 80 / n;
        const size_t blocks_per_tile = sla::TileMap::T / n;
        REQUIRE(dx1 - dx0 == blocks_per_tile);

        REQUIRE(count_value_in_box(prev, new_w, dx0, dx0, dx1, dx1, 255) ==
                blocks_per_tile * blocks_per_tile);                  // the written tile
        REQUIRE(nonzero_outside_box(prev, new_w, new_h, dx0, dx0, dx1, dx1) == 0);

        // And it is still byte for byte what the dense path makes of the real
        // canvas, so "left at zero" is also the correct answer.
        require_sparse_preview_matches(c.poisoned, c.px, c.w, c.h, n, c.tiles);
    }
}

TEST_CASE("Sparse preview clips the last partial tile of an 8K layer",
          "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    // 7536 x 3240 at n = 5: new_w = 1507, so the source columns actually used
    // are [0, 7535) and column 7535 is dropped. Tile column 188 spans source
    // columns [7520, 7536), which maps to target columns [1504, 1507).
    const size_t W = 7536, H = 3240, n = 5;
    size_t new_w = 0, new_h = 0;
    preview_size(W, H, n, new_w, new_h);
    REQUIRE(new_w == 1507);
    REQUIRE(new_h == 648);
    REQUIRE(new_w * n == 7535);                          // one source column left over
    REQUIRE(new_w * n < W);

    SECTION("tile column 188 maps to target columns [1504, 1507)")
    {
        std::vector<uint8_t> px(W * H, 0);
        sla::TileMap tiles(W, H);
        for (size_t y = 0; y < 80; ++y)
            write_span(px, tiles, W, y, 7520, 7536, 255);    // the whole partial tile
        REQUIRE(marked_tiles(tiles) == tile_list({{188, 0}, {188, 1}}));

        const std::vector<uint8_t> poisoned = poison_clean_areas(px, W, H, tiles);
        const std::vector<uint8_t> prev =
            require_sparse_preview_matches(poisoned, px, W, H, n, tiles);

        // Exactly 3 columns x 16 rows of white, and nothing anywhere else.
        REQUIRE(count_value_in_box(prev, new_w, 1504, 0, 1507, 16, 255) == 3 * 16);
        REQUIRE(nonzero_outside_box(prev, new_w, new_h, 1504, 0, 1507, 16) == 0);
    }

    SECTION("source column 7535 never reaches the preview")
    {
        std::vector<uint8_t> px(W * H, 0);
        sla::TileMap tiles(W, H);
        for (size_t y = 0; y < 80; ++y)
            write_span(px, tiles, W, y, W - 1, W, 255);      // column 7535 only
        REQUIRE(marked_tiles(tiles) == tile_list({{188, 0}, {188, 1}}));
        REQUIRE(px[7535] == 255);

        const std::vector<uint8_t> poisoned = poison_clean_areas(px, W, H, tiles);
        const std::vector<uint8_t> prev =
            require_sparse_preview_matches(poisoned, px, W, H, n, tiles);

        // The leftover column is outside [0, new_w * n), so the preview is blank.
        size_t lit = 0;
        for (uint8_t v : prev)
            if (v != 0) ++lit;
        REQUIRE(lit == 0);
    }
}

TEST_CASE("Sparse preview falls back when its conditions do not hold",
          "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    // Every fallback reads the whole canvas, so running it on the poisoned copy
    // has to give what the dense encoder gives for that same poisoned copy --
    // and something other than the sparse answer for the real canvas. That is
    // what tells a fallback apart from a sparse run.
    const PreviewCanvas c = single_tile_canvas();

    auto require_fallback = [&](size_t n, const sla::TileMap &tiles) {
        INFO("n = " << n);
        size_t new_w = 0, new_h = 0;
        preview_size(c.w, c.h, n, new_w, new_h);

        const std::vector<uint8_t> fell_back =
            bytes_of(sparse_preview_png(c.poisoned, c.w, c.h, n, tiles));
        const std::vector<uint8_t> dense_poisoned =
            bytes_of(dense_preview_png(c.poisoned, c.w, c.h, n));
        const std::vector<uint8_t> dense_clean =
            bytes_of(dense_preview_png(c.px, c.w, c.h, n));

        REQUIRE(fell_back.size() == dense_poisoned.size());
        REQUIRE(first_difference(fell_back, dense_poisoned) == NoDifference);  // read it all
        REQUIRE(first_difference(dense_poisoned, dense_clean) != NoDifference);  // poison shows

        // Same thing seen in the pixels: the poisoned clean areas averaged to
        // 0xAA rather than staying at 0.
        const std::vector<uint8_t> prev =
            decode_preview(sparse_preview_png(c.poisoned, c.w, c.h, n, tiles), new_w, new_h);
        REQUIRE(prev[0] == CleanAreaPoison);
    };

    // The fixed-block gate has to hold for these, otherwise the encoder would
    // never reach the sparse downscale and the T % n test would not be the one
    // being exercised.
    auto require_fixed_block_gate = [&](size_t n) {
        INFO("n = " << n);
        size_t new_w = 0, new_h = 0;
        preview_size(c.w, c.h, n, new_w, new_h);
        REQUIRE(1.0 / (1.0 / double(n)) == double(n));
        REQUIRE(new_w * n <= c.w);
        REQUIRE(new_h * n <= c.h);
        REQUIRE(sla::TileMap::T % n != 0);          // ... and this is what fires
    };

    SECTION("a factor that does not divide the tile edge")
    {
        require_fixed_block_gate(32);               // 80 % 32 = 16
        require_fallback(32, c.tiles);
    }

    SECTION("N = 3 and N = 6 are not divisors of the tile edge either")
    {
        require_fixed_block_gate(3);                // 80 % 3 = 2
        require_fallback(3, c.tiles);
        require_fixed_block_gate(6);                // 80 % 6 = 2
        require_fallback(6, c.tiles);
    }

    SECTION("a tile map that does not match the canvas width")
    {
        // The map cannot be trusted to describe this buffer, so the encoder has
        // to read every pixel rather than skip by it.
        sla::TileMap mismatched(c.w / 2, c.h);
        REQUIRE(mismatched.width_px() != c.w);
        require_fallback(10, mismatched);
    }
}


// ---------------------------------------------------------------------------
// Consumers that do not read the tile map (tasks 2.15). The layer PNG encoder
// and read_pixel must return what the buffer holds, tile map or no tile map --
// above all after a partial reset, which is the only thing about the buffer
// that the map changed.
// ---------------------------------------------------------------------------

TEST_CASE("The layer PNG encoder reads the buffer, not the tile map", "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    const size_t W = 400, H = 240;
    sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
    draw_scattered(raster, H);

    const sla::TileMap *tiles = raster.written_tiles();
    REQUIRE(tiles != nullptr);
    const size_t dirty = marked_tiles(*tiles).size();
    REQUIRE(dirty > 0);
    REQUIRE(dirty < tiles->tiles_x() * tiles->tiles_y());    // clean tiles do exist

    // Through the raster, which carries the map, and on a bare copy of the
    // pixels, where no map exists at all.
    const std::vector<uint8_t> px = raster_pixels(raster, W, H);
    const sla::EncodedRaster through_raster = layer_png(raster);
    const sla::EncodedRaster from_buffer    = layer_png_of_buffer(px, W, H);

    const std::vector<uint8_t> a = bytes_of(through_raster);
    const std::vector<uint8_t> b = bytes_of(from_buffer);
    REQUIRE(a.size() == b.size());
    REQUIRE(first_difference(a, b) == NoDifference);

    // The PNG really carries the whole layer, clean tiles included.
    const std::vector<uint8_t> decoded = decode_layer_png(through_raster, W, H);
    REQUIRE(first_difference(decoded, px) == NoDifference);
    require_read_pixel_matches_png(raster, decoded, W, H);

    size_t lit = 0;
    for (uint8_t v : decoded)
        if (v != 0) ++lit;
    REQUIRE(lit > 0);                                        // not a blank layer
    REQUIRE(lit < decoded.size());
}

TEST_CASE("A reused raster's layer PNG and read_pixel match a fresh one",
          "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    const size_t W = 400, H = 240;

    // The reused raster carries a full plate first, so every tile is written and
    // the partial reset has the largest possible job to do.
    sla::RasterGrayscaleAAGammaPower reused({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
    reused.draw_binary(pixel_rect(H, 0, 0, W, H));
    REQUIRE(marked_tiles(*reused.written_tiles()).size() ==
            reused.written_tiles()->tiles_x() * reused.written_tiles()->tiles_y());
    REQUIRE(reused.read_pixel(200, 120) == 255);

    reused.reset();

    auto second_layer = [&](sla::RasterGrayscaleAA &r) {
        r.draw_binary(pixel_rect(H, 10, 10, 40, 40));
        r.draw(pixel_polygon(H, {{300.5, 190.25}, {340.75, 200.5}, {310.25, 225.5}}));
    };
    second_layer(reused);

    sla::RasterGrayscaleAAGammaPower fresh({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
    second_layer(fresh);

    // Same tiles, same layer file, same pixels.
    REQUIRE(marked_tiles(*reused.written_tiles()) == marked_tiles(*fresh.written_tiles()));

    const std::vector<uint8_t> a = bytes_of(layer_png(reused));
    const std::vector<uint8_t> b = bytes_of(layer_png(fresh));
    REQUIRE(a.size() == b.size());
    REQUIRE(first_difference(a, b) == NoDifference);

    require_read_pixel_agrees_in_window(reused, fresh, 0, 0, W, H);

    // The middle of the plate was white in the first layer and is in a clean
    // tile now: read_pixel has to say 0, exactly as it does for the fresh one.
    REQUIRE(reused.read_pixel(200, 120) == 0);
    REQUIRE(fresh.read_pixel(200, 120) == 0);

    const std::vector<uint8_t> decoded = decode_layer_png(layer_png(reused), W, H);
    require_read_pixel_matches_png(reused, decoded, W, H);
    REQUIRE(decoded[120 * W + 200] == 0);                    // no residue in the layer file
}

TEST_CASE("read_pixel after a partial reset of an 8K layer", "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    // 8K is 24 million pixels, so this one samples: a grid whose step shares no
    // factor with the 80 pixel tile edge, plus dense windows where something was
    // actually drawn (X3). The layer PNG is compared at 400 x 240 above; at this
    // size two deflate passes would dominate the run time for nothing new.
    const size_t W = 7536, H = 3240;

    auto second_layer = [&](sla::RasterGrayscaleAA &r) {
        r.draw_binary(pixel_rect(H, W - 1, H - 1, W, H));     // the partial corner tile
        r.draw_binary(pixel_rect(H, 3040, 1520, 3080, 1560)); // wholly inside tile (76, 38)
    };

    sla::RasterGrayscaleAAGammaPower reused({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
    reused.draw_binary(pixel_rect(H, 0, 0, W, H));            // full plate first
    REQUIRE(reused.read_pixel(0, 0) == 255);
    REQUIRE(reused.read_pixel(W - 1, H - 1) == 255);

    reused.reset();
    second_layer(reused);

    sla::RasterGrayscaleAAGammaPower fresh({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
    second_layer(fresh);

    REQUIRE(marked_tiles(*reused.written_tiles()) == marked_tiles(*fresh.written_tiles()));
    REQUIRE(marked_tiles(*reused.written_tiles()) == tile_list({{76, 38}, {188, 80}}));

    require_read_pixel_agrees_on_grid(reused, fresh, W, H, 137);   // 137 is prime, 80 is not
    require_read_pixel_agrees_in_window(reused, fresh, W - 4, H - 4, W, H);
    require_read_pixel_agrees_in_window(reused, fresh, 3036, 1516, 3084, 1564);
    require_read_pixel_agrees_in_window(reused, fresh, 0, 0, 4, 4);

    // What the second layer drew is there, and what the first one drew is gone.
    REQUIRE(reused.read_pixel(W - 1, H - 1) == 255);
    REQUIRE(reused.read_pixel(3070, 1550) == 255);
    REQUIRE(reused.read_pixel(0, 0) == 0);
    REQUIRE(reused.read_pixel(4000, 2000) == 0);
}


// ---------------------------------------------------------------------------
// Verify mode (design D8, tasks 2.16). SLA_RASTER_VERIFY is read once per
// process, so these tests drive the scan through
// test_only_verify_written_tiles(), which is the very same code with the gate
// left out, and use the gated verify_written_tiles() to show that the gate
// short-circuits. Both violations are built by hand: no drawing path can
// produce them, which is the whole point of the check.
//
// Tests 2 and 3 print a [raster-verify] line to stderr on purpose -- that is
// the report being tested. A passing run shows those lines.
// ---------------------------------------------------------------------------

TEST_CASE("Verify mode accepts a correctly marked raster", "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    const size_t W = 400, H = 240;
    sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
    draw_scattered(raster, H);

    const sla::TileMap *tiles = raster.written_tiles();
    REQUIRE(tiles != nullptr);
    require_marks_cover_pixels(raster_pixels(raster, W, H), W, H, *tiles);
    require_row_summaries_consistent(*tiles);

    REQUIRE_NOTHROW(raster.test_only_verify_written_tiles(7));
    REQUIRE_NOTHROW(raster.verify_written_tiles(7));

    // A layer that was reset is clean everywhere, which the scan also accepts.
    raster.reset();
    REQUIRE(marked_tiles(*tiles).empty());
    REQUIRE_NOTHROW(raster.test_only_verify_written_tiles(8));

    // And so is an over-marked one: marking more than was written is allowed.
    raster.draw_binary(pixel_rect(H, 100, 100, 104, 104));
    REQUIRE_NOTHROW(raster.test_only_verify_written_tiles(9));
}

TEST_CASE("Verify mode catches a written pixel in a clean tile", "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    const size_t W = 400, H = 240;
    sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
    raster.draw_binary(pixel_rect(H, 170, 90, 200, 120));        // tile (4, 2)

    // Wiping the map leaves the pixels exactly where they are, so every one of
    // them now sits in a tile nobody marked. That is the missed marking the
    // check exists to find, and reset() and the sparse consumers would skip it.
    sla::TileMap *tiles = const_cast<sla::TileMap *>(raster.written_tiles());
    REQUIRE(tiles != nullptr);
    REQUIRE(marked_tiles(*tiles) == tile_list({{4, 2}}));
    REQUIRE(raster.read_pixel(180, 100) == 255);
    REQUIRE_NOTHROW(raster.test_only_verify_written_tiles(41));   // sound before the wipe

    tiles->clear();
    REQUIRE(marked_tiles(*tiles).empty());

    REQUIRE_THROWS_AS(raster.test_only_verify_written_tiles(42), Slic3r::RuntimeError);

    // The right branch fired, not the row summary one.
    std::string message;
    try {
        raster.test_only_verify_written_tiles(42);
    } catch (const Slic3r::RuntimeError &e) {
        message = e.what();
    }
    REQUIRE(message.find("written pixel in a clean tile") != std::string::npos);
}

TEST_CASE("Verify mode catches a marked tile whose row summary is clear",
          "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    // Nothing is drawn here, so no stray pixel can trip the other branch first
    // and the N2 violation is the only thing left to find.
    const size_t W = 400, H = 240;
    sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);

    sla::TileMap *tiles = const_cast<sla::TileMap *>(raster.written_tiles());
    REQUIRE(tiles != nullptr);
    REQUIRE(marked_tiles(*tiles).empty());
    REQUIRE_NOTHROW(raster.test_only_verify_written_tiles(1));    // sound to start with

    tiles->test_only_mark_tile_without_row(2, 1);
    REQUIRE(tiles->written(2, 1));
    REQUIRE_FALSE(tiles->row_written(1));                        // the inconsistency
    REQUIRE(marked_tiles(*tiles) == tile_list({{2, 1}}));

    REQUIRE_THROWS_AS(raster.test_only_verify_written_tiles(5), Slic3r::RuntimeError);

    std::string message;
    try {
        raster.test_only_verify_written_tiles(5);
    } catch (const Slic3r::RuntimeError &e) {
        message = e.what();
    }
    REQUIRE(message.find("inconsistent tile map") != std::string::npos);

    // Setting the summary makes the same map sound again: the tile is simply
    // over-marked, which is allowed.
    tiles->mark_span(80, 40, 40);
    REQUIRE(tiles->row_written(1));
    REQUIRE_NOTHROW(raster.test_only_verify_written_tiles(6));
}

TEST_CASE("Verify mode does nothing when the environment switch is off",
          "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    if (sla::raster_verify_enabled()) {
        WARN("SLA_RASTER_VERIFY=1 is set; skipping the switched-off test.");
        return;
    }

    const size_t W = 400, H = 240;
    sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
    raster.draw_binary(pixel_rect(H, 170, 90, 200, 120));

    // Both violations at once: written pixels with no tile marked, and a marked
    // tile whose row summary is clear.
    sla::TileMap *tiles = const_cast<sla::TileMap *>(raster.written_tiles());
    REQUIRE(tiles != nullptr);
    tiles->clear();
    tiles->test_only_mark_tile_without_row(4, 2);

    // The scan rejects it ...
    REQUIRE_THROWS_AS(raster.test_only_verify_written_tiles(0), Slic3r::RuntimeError);

    // ... and the gated entry point returns without looking, which is what the
    // engine does on every layer unless SLA_RASTER_VERIFY=1 is set.
    REQUIRE_NOTHROW(raster.verify_written_tiles(0));
    REQUIRE_NOTHROW(raster.verify_written_tiles(1));
}


// ---------------------------------------------------------------------------
// Random differential test (tasks 2.17). Fixed seeds, so every run draws the
// same layers and a failure can be reproduced. Each layer goes through the
// whole pipeline -- layer file RLE and preview downscale at all four factors --
// and has to come out byte for byte as the dense code makes it, while the
// poisoned clean areas prove the sparse paths never read them.
// ---------------------------------------------------------------------------

TEST_CASE("Random layers match the dense pipeline byte for byte", "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    struct Canvas {
        size_t      w, h, polygons;
        double      r_min, r_max;
        unsigned    seed;
        const char *name;
    };

    // 400 x 240 is a whole number of tiles on both axes; 803 x 623 is not --
    // 803 = 10 * 80 + 3 and 623 = 7 * 80 + 63 -- so the right column of tiles is
    // 3 pixels wide and the bottom row 63 pixels tall.
    const std::vector<Canvas> canvases = {
        {400, 240, 8, 2.0, 20.0, 42u, "400 x 240, whole tiles"},
        {803, 623, 12, 2.0, 40.0, 1337u, "803 x 623, partial tiles on both edges"},
    };

    bool saw_clean_tiles = false;

    for (const Canvas &c : canvases) {
        INFO(c.name);
        sla::RasterGrayscaleAAGammaPower raster({c.w, c.h}, {TileMmPerPx, TileMmPerPx}, {}, 1.);

        // Three layers on one raster, each after a reset, so the partial reset
        // is in the loop too: a wrong clear would show up as a byte difference
        // in the next layer.
        for (unsigned layer = 0; layer < 3; ++layer) {
            INFO("layer " << layer);
            raster.reset();
            for (const ExPolygon &poly :
                 random_polygons(c.w, c.h, c.seed + layer, c.polygons, c.r_min, c.r_max))
                raster.draw(poly);

            if (require_random_layer_matches(raster, c.w, c.h)) saw_clean_tiles = true;
        }
    }

    // If every layer had covered every tile, the sparse paths would have had
    // nothing to skip and the comparisons above would prove very little.
    REQUIRE(saw_clean_tiles);
}

TEST_CASE("A random 8K layer matches the dense pipeline byte for byte",
          "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    // 7536 x 3240 is 95 x 41 tiles with both edges partial. One layer only: the
    // buffer is 24 million pixels and the reference walks all of them for the
    // layer file and once more per preview factor, so a second layer would buy
    // coverage that the small canvases already give (X3).
    const size_t W = 7536, H = 3240;
    sla::RasterGrayscaleAAGammaPower raster({W, H}, {TileMmPerPx, TileMmPerPx}, {}, 1.);

    for (const ExPolygon &poly : random_polygons(W, H, 2024u, 14, 20.0, 300.0))
        raster.draw(poly);

    const sla::TileMap *tiles = raster.written_tiles();
    REQUIRE(tiles != nullptr);
    REQUIRE(tiles->tiles_x() == 189);
    REQUIRE(tiles->tiles_y() == 81);

    REQUIRE(require_random_layer_matches(raster, W, H));      // clean tiles remain

    // And the same raster reused for an empty layer leaves nothing behind.
    raster.reset();
    REQUIRE(marked_tiles(*tiles).empty());
    REQUIRE(raster.read_pixel(0, 0) == 0);
    REQUIRE(raster.read_pixel(W - 1, H - 1) == 0);
}

// ---------------------------------------------------------------------------
// Fixed-block preview kernel (design D13, tasks 2.27).
//
// n = 4, 5, 8 and 10 downscale through a kernel that sums each block with
// _mm_sad_epu8, or byte by byte on builds without SSE2. Every test compares
// with preview_box_downscale_integer_reference() and runs the kernel both ways
// through sla::test_only_preview_box_downscale_integer(), so the scalar kernel
// is exercised on x86-64 as well.
// ---------------------------------------------------------------------------

namespace {

const size_t FixedBlockFactors[] = {4, 5, 8, 10};

// The reference downscale of src, required equal byte for byte to the engine
// kernel and to the scalar kernel (both through the test entry) and, unless
// through_encoder is false, to the pixels PNGPreviewEncoder writes. Returns the
// reference pixels.
std::vector<uint8_t> require_fixed_block_kernels_match(const uint8_t *src, size_t w, size_t h,
                                                       size_t n, bool through_encoder = true)
{
    INFO("w = " << w << ", h = " << h << ", n = " << n);

    size_t new_w = 0, new_h = 0;
    preview_size(w, h, n, new_w, new_h);
    REQUIRE(new_w == w / n);          // whole blocks only, as the fixed-block guard needs
    REQUIRE(new_h == h / n);
    REQUIRE(new_w >= 1);
    REQUIRE(new_h >= 1);

    std::vector<uint8_t> expected(new_w * new_h);
    sla::preview_box_downscale_integer_reference(src, w, expected.data(), new_w, new_h, 1, n);

    for (bool scalar : {false, true}) {
        INFO((scalar ? "scalar kernel" : "engine kernel"));
        // Every pixel starts at the complement of its expected value, so one the
        // kernel never writes cannot match by accident.
        std::vector<uint8_t> out(expected.size());
        for (size_t i = 0; i < out.size(); ++i) out[i] = uint8_t(~expected[i]);

        sla::test_only_preview_box_downscale_integer(src, w, out.data(), new_w, new_h, n, scalar);
        REQUIRE(first_difference(out, expected) == NoDifference);
    }

    if (through_encoder) {
        INFO("PNGPreviewEncoder");
        const std::vector<uint8_t> encoded =
            decode_preview(sla::PNGPreviewEncoder{1.0 / double(n)}(src, w, h, 1), new_w, new_h);
        REQUIRE(first_difference(encoded, expected) == NoDifference);
    }
    return expected;
}

// Sets the n x n block (bx, by) so that its bytes add up to `sum`: as many 255s
// as fit, then the remainder, then zeros, in row-major order within the block.
// `reversed` lays them out from the block's last byte backwards, so the
// partial byte lands near both ends across the blocks of one test.
void fill_block_with_sum(std::vector<uint8_t> &px, size_t w, size_t n, size_t bx, size_t by,
                         unsigned sum, bool reversed)
{
    const size_t area = n * n;
    for (size_t j = 0; j < area; ++j) {
        const unsigned before = unsigned(255 * j);        // held by the bytes placed so far
        uint8_t v = 0;
        if (sum >= before + 255)
            v = 255;
        else if (sum > before)
            v = uint8_t(sum - before);

        const size_t k = reversed ? area - 1 - j : j;
        px[(by * n + k / n) * w + bx * n + k % n] = v;
    }
}

// `size` readable bytes placed so that the byte just past their end
// (Side::End) or just before their start (Side::Start) lies on a page the
// process may not touch. A read one byte too far faults and ends the test run
// instead of passing silently.
class GuardedBuffer
{
public:
    enum class Side { Start, End };

    GuardedBuffer(size_t size, Side side)
    {
        const size_t page = page_size();
        if (page == 0 || size == 0) return;

        const size_t readable = (size + page - 1) / page * page;
        m_total = readable + page;
        if (!reserve()) return;

        uint8_t *const guard = side == Side::End ? m_base + readable : m_base;
        if (!protect(guard, page)) return;

        m_data = side == Side::End ? m_base + readable - size : m_base + page;
    }

    ~GuardedBuffer() { release(); }

    GuardedBuffer(const GuardedBuffer &)            = delete;
    GuardedBuffer &operator=(const GuardedBuffer &) = delete;

    // Null when this platform has no page protection or setting it up failed.
    uint8_t *data() const { return m_data; }

private:
#if defined(_WIN32)
    static size_t page_size()
    {
        SYSTEM_INFO info;
        GetSystemInfo(&info);
        return size_t(info.dwPageSize);
    }
    bool reserve()
    {
        m_base = static_cast<uint8_t *>(
            VirtualAlloc(nullptr, m_total, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE));
        return m_base != nullptr;
    }
    static bool protect(uint8_t *p, size_t len)
    {
        DWORD old = 0;
        return VirtualProtect(p, len, PAGE_NOACCESS, &old) != 0;
    }
    void release()
    {
        if (m_base != nullptr) VirtualFree(m_base, 0, MEM_RELEASE);
    }
#elif defined(__unix__) || defined(__APPLE__)
    static size_t page_size()
    {
        const long p = sysconf(_SC_PAGESIZE);
        return p > 0 ? size_t(p) : 0;
    }
    bool reserve()
    {
        void *p = mmap(nullptr, m_total, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, -1, 0);
        if (p == MAP_FAILED) return false;
        m_base = static_cast<uint8_t *>(p);
        return true;
    }
    static bool protect(uint8_t *p, size_t len) { return mprotect(p, len, PROT_NONE) == 0; }
    void release()
    {
        if (m_base != nullptr) munmap(m_base, m_total);
    }
#else
    static size_t page_size() { return 0; }
    bool reserve() { return false; }
    static bool protect(uint8_t *, size_t) { return false; }
    void release() {}
#endif

    uint8_t *m_base  = nullptr;
    size_t   m_total = 0;
    uint8_t *m_data  = nullptr;
};

} // namespace

TEST_CASE("Fixed-block preview kernel matches reference for every block value", "[raster-scan]")
{
    for (size_t n : FixedBlockFactors) {
        // Two rows of 256 uniform blocks: value v left to right, then 255 - v,
        // so every value has different neighbours in the two rows.
        const size_t W = 256 * n, H = 2 * n;
        std::vector<uint8_t> px(W * H);
        for (size_t y = 0; y < H; ++y)
            for (size_t x = 0; x < W; ++x) {
                const size_t v = x / n;
                px[y * W + x] = uint8_t(y < n ? v : 255 - v);
            }

        const std::vector<uint8_t> prev = require_fixed_block_kernels_match(px.data(), W, H, n);

        // A uniform block averages to its own value.
        size_t wrong = 0;
        for (size_t dx = 0; dx < 256; ++dx) {
            if (prev[dx] != dx) ++wrong;
            if (prev[256 + dx] != 255 - dx) ++wrong;
        }
        INFO("n = " << n);
        REQUIRE(wrong == 0);
    }
}

TEST_CASE("Fixed-block preview kernel divides every possible block sum like the reference",
          "[raster-scan]")
{
    for (size_t n : FixedBlockFactors) {
        // One row of blocks, block bx summing to bx: every sum from 0 to
        // 255 n^2 (25,501 of them for n = 10) goes through the constant divisor.
        const size_t area   = n * n;
        const size_t blocks = 255 * area + 1;
        const size_t W = blocks * n, H = n;
        std::vector<uint8_t> px(W * H, 0);
        for (size_t bx = 0; bx < blocks; ++bx)
            fill_block_with_sum(px, W, n, bx, 0, unsigned(bx), bx % 2 == 1);

        const std::vector<uint8_t> prev = require_fixed_block_kernels_match(px.data(), W, H, n);

        size_t wrong = 0;
        for (size_t bx = 0; bx < blocks; ++bx)
            if (prev[bx] != bx / area) ++wrong;
        INFO("n = " << n);
        REQUIRE(wrong == 0);
    }
}

TEST_CASE("Fixed-block preview kernel attributes every pixel to its own block", "[raster-scan]")
{
    for (size_t n : FixedBlockFactors) {
        // 3 x 2 blocks of 100, except that each block's first pixel is 99, so
        // every block sums to 100 n^2 - 1 and averages to 99. Adding 1 to any one
        // pixel lifts exactly its own block to 100: a byte counted in the wrong
        // block, or not counted, changes the output.
        const size_t W = 3 * n, H = 2 * n;
        std::vector<uint8_t> base(W * H, 100);
        for (size_t by = 0; by < 2; ++by)
            for (size_t bx = 0; bx < 3; ++bx)
                base[by * n * W + bx * n] = 99;

        for (size_t y = 0; y < H; ++y)
            for (size_t x = 0; x < W; ++x) {
                INFO("pixel (" << x << ", " << y << ")");
                std::vector<uint8_t> px = base;
                ++px[y * W + x];

                const std::vector<uint8_t> prev =
                    require_fixed_block_kernels_match(px.data(), W, H, n);

                const size_t lifted = (y / n) * 3 + x / n;
                size_t wrong = 0;
                for (size_t i = 0; i < prev.size(); ++i)
                    if (prev[i] != (i == lifted ? 100 : 99)) ++wrong;
                REQUIRE(wrong == 0);
            }
    }
}

TEST_CASE("Fixed-block preview kernel matches reference on stripes that are not multiples of 8",
          "[raster-scan]")
{
    for (size_t n : FixedBlockFactors)
        for (size_t period : {size_t(3), size_t(7), size_t(11)})
            for (bool diagonal : {false, true})
                // Whole blocks only, and n - 1 extra columns and rows past the
                // last whole block, which the fast path must leave unread.
                for (size_t extra : {size_t(0), n - 1}) {
                    INFO("period " << period << (diagonal ? ", diagonal" : ", vertical")
                                   << ", extra " << extra);
                    const size_t W = 24 * n + extra, H = 6 * n + extra;
                    std::vector<uint8_t> px(W * H);
                    for (size_t y = 0; y < H; ++y)
                        for (size_t x = 0; x < W; ++x) {
                            const size_t phase = (diagonal ? x + 2 * y : x) % period;
                            px[y * W + x] = phase == 0 ? uint8_t(255) : uint8_t(19 * phase);
                        }
                    require_fixed_block_kernels_match(px.data(), W, H, n);
                }
}

TEST_CASE("Fixed-block preview kernel does not depend on where the buffer starts",
          "[raster-scan]")
{
    std::mt19937 rng(20260917u);

    for (size_t n : FixedBlockFactors) {
        const size_t W = 13 * n + 3, H = 4 * n + 1;
        std::vector<uint8_t> px(W * H);
        for (uint8_t &v : px)
            v = rng() % 3 == 0 ? uint8_t(0) : uint8_t(rng() % 256);

        // The same pixels copied to 16 successive offsets, so the first byte
        // sits at every alignment modulo 16.
        std::vector<uint8_t> storage(W * H + 16, 0);
        std::vector<uint8_t> first;
        for (size_t offset = 0; offset < 16; ++offset) {
            INFO("n = " << n << ", offset " << offset);
            std::fill(storage.begin(), storage.end(), uint8_t(0));
            std::copy(px.begin(), px.end(), storage.begin() + std::ptrdiff_t(offset));

            const std::vector<uint8_t> prev =
                require_fixed_block_kernels_match(storage.data() + offset, W, H, n);
            if (offset == 0)
                first = prev;
            else
                REQUIRE(first_difference(prev, first) == NoDifference);
        }
    }
}

TEST_CASE("Fixed-block preview kernel never reads past either end of the buffer",
          "[raster-scan]")
{
    for (GuardedBuffer::Side side : {GuardedBuffer::Side::End, GuardedBuffer::Side::Start})
        for (size_t n : FixedBlockFactors)
            for (size_t blocks_x : {size_t(1), size_t(3), size_t(7), size_t(13)})
                for (size_t blocks_y : {size_t(1), size_t(2), size_t(5)}) {
                    INFO((side == GuardedBuffer::Side::End ? "guard after" : "guard before")
                         << ", n = " << n << ", " << blocks_x << " x " << blocks_y << " blocks");

                    // Whole blocks only: the last block read ends on the buffer's
                    // last byte and the first starts on its first byte, so the
                    // guard page is one byte away from a block that is read.
                    const size_t W = blocks_x * n, H = blocks_y * n;
                    GuardedBuffer buf(W * H, side);
                    if (buf.data() == nullptr) {
                        WARN("No guard page available on this platform; skipping.");
                        return;
                    }
                    std::fill(buf.data(), buf.data() + W * H, uint8_t(255));   // a full plate

                    const std::vector<uint8_t> prev =
                        require_fixed_block_kernels_match(buf.data(), W, H, n);
                    REQUIRE(size_t(std::count(prev.begin(), prev.end(), uint8_t(255))) ==
                            prev.size());

                    // The tile-aware encoder over the same buffer, every tile
                    // written, reads the same blocks.
                    sla::TileMap tiles(W, H);
                    for (size_t y = 0; y < H; ++y)
                        tiles.mark_span(0, y, W);
                    const std::vector<uint8_t> sparse = decode_preview(
                        sla::SparsePNGPreviewEncoder{1.0 / double(n)}(buf.data(), W, H, 1, tiles),
                        W / n, H / n);
                    REQUIRE(first_difference(sparse, prev) == NoDifference);
                }
}

TEST_CASE("Previews of an anti-aliased full-plate slab match the reference",
          "[raster-scan][tiles]")
{
    if (!tile_tracking_available()) return;

    struct Slab {
        size_t w, h, n;
        std::vector<std::pair<double, double>> corners;   // pixels, y down
        const char *name;
    };

    // Slightly rotated quads over most of the plate, like fullplate-16k-slab:
    // anti-aliased along the whole perimeter, solid 255 inside, and a margin of
    // clean tiles on every side.
    const std::vector<Slab> slabs = {
        {15120, 6230, 10,
         {{417.25, 220.5}, {14703.75, 231.25}, {14698.5, 6009.5}, {411.75, 5998.25}},
         "16K, n = 10"},
        {7536, 3240, 5,
         {{208.25, 110.5}, {7328.75, 116.25}, {7326.5, 3120.5}, {205.75, 3114.25}},
         "8K, n = 5"},
    };

    for (const Slab &s : slabs) {
        INFO(s.name);
        sla::RasterGrayscaleAAGammaPower raster({s.w, s.h}, {TileMmPerPx, TileMmPerPx}, {}, 1.);
        raster.draw(pixel_polygon(s.h, s.corners));

        const sla::TileMap *tiles = raster.written_tiles();
        REQUIRE(tiles != nullptr);

        const std::vector<uint8_t> px = raster_pixels(raster, s.w, s.h);
        require_marks_cover_pixels(px, s.w, s.h, *tiles);    // I2 holds, so poisoning is safe

        const size_t total = tiles->tiles_x() * tiles->tiles_y();
        const size_t dirty = marked_tiles(*tiles).size();
        REQUIRE(dirty * 10 > total * 8);      // mostly written, like the real slab
        REQUIRE(dirty < total);               // with clean tiles left to skip

        // Sparse encoder on the poisoned copy, dense encoder and reference on the
        // real canvas: pixel for pixel, and the two PNG streams byte for byte.
        const std::vector<uint8_t> poisoned = poison_clean_areas(px, s.w, s.h, *tiles);
        const std::vector<uint8_t> prev =
            require_sparse_preview_matches(poisoned, px, s.w, s.h, s.n, *tiles);

        // Both kernels on the real canvas; the encoder was covered just above.
        const std::vector<uint8_t> kernels =
            require_fixed_block_kernels_match(px.data(), s.w, s.h, s.n, false);
        REQUIRE(first_difference(kernels, prev) == NoDifference);

        size_t white = 0, gray = 0;
        for (uint8_t v : prev) {
            if (v == 255) ++white;
            else if (v != 0) ++gray;
        }
        REQUIRE(white * 10 > prev.size() * 8);   // solid interior
        REQUIRE(gray > 0);                       // averaged anti-aliased edges
    }
}

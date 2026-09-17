///|/ Copyright (c) Prusa Research 2020 - 2022 Tomáš Mészáros @tamasmeszaros, Vojtěch Bubník @bubnikv
///|/ Copyright (c) 2022 ole00 @ole00
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef SLA_RASTERBASE_HPP
#define SLA_RASTERBASE_HPP

#include <libslic3r/ExPolygon.hpp>
#include <stddef.h>
#include <algorithm>
#include <cassert>
#include <cstring>
#include <ostream>
#include <memory>
#include <vector>
#include <array>
#include <utility>
#include <cstdint>
#include <functional>
#include <string>
#include <cstddef>

#include "libslic3r/Point.hpp"
#include "libslic3r/libslic3r.h"

namespace Slic3r {

namespace sla {

// Raw byte buffer paired with its size. Suitable for compressed image data.
class EncodedRaster {
protected:
    std::vector<uint8_t> m_buffer;
    std::string m_ext;
public:
    EncodedRaster() = default;
    explicit EncodedRaster(std::vector<uint8_t> &&buf, std::string ext)
        : m_buffer(std::move(buf)), m_ext(std::move(ext))
    {}
    
    size_t size() const { return m_buffer.size(); }
    const void * data() const { return m_buffer.data(); }
    const char * extension() const { return m_ext.c_str(); }
};

/// Type that represents a resolution in pixels.
struct Resolution {
    size_t width_px = 0;
    size_t height_px = 0;

    Resolution() = default;
    Resolution(size_t w, size_t h) : width_px(w), height_px(h) {}
    size_t pixels() const { return width_px * height_px; }
};

/// Types that represents the dimension of a pixel in millimeters.
struct PixelDim {
    double w_mm = 1.;
    double h_mm = 1.;

    PixelDim() = default;
    PixelDim(double px_width_mm, double px_height_mm)
        : w_mm(px_width_mm), h_mm(px_height_mm)
    {}
};

using RasterEncoder =
    std::function<EncodedRaster(const void *ptr, size_t w, size_t h, size_t num_components)>;

// Sibling of RasterEncoder: mutates the raster's pixel buffer in place (no
// encoding, no return). Used to inject format-specific post-processing (e.g.
// SL1's AA quantization + blur) into the generic raster without leaking the
// recipe into the raster core.
using RasterPostProcessor =
    std::function<void(void *ptr, size_t w, size_t h, size_t num_components)>;

// A post-processor plus what the raster may assume about it. The raster
// cannot see inside fn, so the property is declared by whoever builds it.
struct RasterPostProcess {
    RasterPostProcessor fn;
    // True only if every output pixel depends on that pixel alone and fn maps
    // the zero background to itself. The raster may then run fn over the
    // written tiles only, one row segment per call, and leave clean tiles
    // clean. False (the safe default) runs fn over the whole buffer once and
    // marks every tile written, since fn may write anywhere (e.g. blur).
    bool zero_preserving_pixel_local = false;
};

// Coarse record of which parts of a raster buffer have been written: the
// buffer is split into T x T pixel tiles, one byte per tile (0 = clean,
// 1 = written), plus one byte per tile row saying whether any tile in it is
// written. Tiles on the right and bottom edges are clipped to the buffer.
//
// Invariants the owning raster must keep once consumers read the map:
//   I1: a clean tile holds only background pixels;
//   I2: a pixel's tile is marked before the pixel is written. Marking more
//       than was written is allowed, missing a written pixel is not.
//
// Bytes rather than a bitset: 16K is 378 x 156 = 58,968 bytes (about 59 KB),
// negligible next to its 94 MB buffer, and a byte per tile keeps marking a run
// of tiles a memset and reading one a plain index.
//
// No locks or atomics: the map is a member of its raster, and a raster is only
// ever used by the thread it is bound to (see AGGRaster::draw_binary()).
class TileMap {
public:
    // Tile side in pixels. A multiple of 8 so tile edges line up with the
    // 8-byte words of the RLE and preview scans, and of 40 so every preview
    // block (n = 4, 5, 8, 10) lies within a single tile.
    static constexpr size_t T = 40;

    TileMap() = default;
    TileMap(size_t width_px, size_t height_px);

    size_t width_px() const { return m_width_px; }
    size_t height_px() const { return m_height_px; }
    size_t tiles_x() const { return m_tiles_x; }
    size_t tiles_y() const { return m_tiles_y; }

    bool written(size_t tx, size_t ty) const
    {
        assert(tx < m_tiles_x && ty < m_tiles_y);
        return m_tile[ty * m_tiles_x + tx] != 0;
    }

    bool row_written(size_t ty) const
    {
        assert(ty < m_tiles_y);
        return m_row_any[ty] != 0;
    }

    // Mark the tiles holding pixels [x, x + len) of pixel row y as written.
    // Sets the tiles and their row summary together: for_each_written_span()
    // skips a row whose summary is clear, so marking one without the other
    // would hide written pixels.
    void mark_span(size_t x, size_t y, size_t len)
    {
        assert(len > 0 && x + len <= m_width_px && y < m_height_px);
        const size_t ty  = y / T;
        const size_t tx0 = x / T;
        const size_t tx1 = (x + len - 1) / T; // < tiles_x since x + len <= W
        std::memset(m_tile.data() + ty * m_tiles_x + tx0, 1, tx1 - tx0 + 1);
        m_row_any[ty] = 1;
    }

    // Mark every tile as written, for writes that bypass mark_span().
    void mark_all()
    {
        std::fill(m_tile.begin(), m_tile.end(), uint8_t(1));
        std::fill(m_row_any.begin(), m_row_any.end(), uint8_t(1));
    }

    // Mark every tile as clean. Only valid once the whole buffer holds the
    // background value, otherwise I1 breaks.
    void clear()
    {
        std::fill(m_tile.begin(), m_tile.end(), uint8_t(0));
        std::fill(m_row_any.begin(), m_row_any.end(), uint8_t(0));
    }

    // Test only (tasks 2.16): marks a tile and deliberately leaves its row
    // summary clear. That is the N2 violation SLA_RASTER_VERIFY has to catch,
    // and no real marking path can produce it -- mark_span() and mark_all()
    // both set the summary -- so a unit test has no other way to build it.
    // Nothing in the engine calls this.
    void test_only_mark_tile_without_row(size_t tx, size_t ty)
    {
        assert(tx < m_tiles_x && ty < m_tiles_y);
        m_tile[ty * m_tiles_x + tx] = 1;
    }

    // Half-open pixel interval [begin, end).
    struct PixelSpan {
        size_t begin;
        size_t end;
    };

    // The only tile-to-pixel conversions. Every consumer of the map, reset()
    // included, takes its pixel ranges from these two, so the clipping of the
    // right and bottom edge tiles lives in one place and holds on both axes
    // (portrait swaps which axis is incomplete).

    // Columns covered by the tiles [tx0, tx1) of one tile row:
    // [tx0 * T, min(tx1 * T, W)).
    PixelSpan pixel_columns(size_t tx0, size_t tx1) const
    {
        assert(tx0 < tx1 && tx1 <= m_tiles_x);
        return {tx0 * T, std::min(tx1 * T, m_width_px)};
    }

    // Rows covered by tile row ty: [ty * T, min((ty + 1) * T, H)).
    PixelSpan pixel_rows(size_t ty) const
    {
        assert(ty < m_tiles_y);
        return {ty * T, std::min((ty + 1) * T, m_height_px)};
    }

    // Call fn(PixelSpan columns) once for each maximal run of consecutive
    // written tiles in tile row ty, left to right. Runs never touch, so the
    // columns between two calls (and before the first and after the last, up
    // to W) are clean. Does nothing when the row has no written tile.
    template<class Fn> void for_each_written_span(size_t ty, Fn &&fn) const
    {
        assert(ty < m_tiles_y);
        if (!row_written(ty))
            return;

        const uint8_t *row = m_tile.data() + ty * m_tiles_x;
        size_t tx = 0;
        while (tx < m_tiles_x) {
            if (row[tx] == 0) {
                ++tx;
                continue;
            }
            size_t tx_end = tx + 1;
            while (tx_end < m_tiles_x && row[tx_end] != 0)
                ++tx_end;
            fn(pixel_columns(tx, tx_end));
            tx = tx_end;
        }
    }

private:
    size_t m_width_px = 0;
    size_t m_height_px = 0;
    size_t m_tiles_x = 0;
    size_t m_tiles_y = 0;
    std::vector<uint8_t> m_tile;    // tiles_x * tiles_y, row-major
    std::vector<uint8_t> m_row_any; // tiles_y
};

static_assert(TileMap::T % 8 == 0, "tile edges must align with 8-byte words");
static_assert(TileMap::T % 40 == 0, "every preview block (n = 4, 5, 8, 10) must fit in one tile");

class RasterBase {
public:
    
    enum Orientation { roLandscape, roPortrait };
    
    using TMirroring = std::array<bool, 2>;
    static const constexpr TMirroring NoMirror = {false, false};
    static const constexpr TMirroring MirrorX  = {true, false};
    static const constexpr TMirroring MirrorY  = {false, true};
    static const constexpr TMirroring MirrorXY = {true, true};
    
    struct Trafo {
        bool mirror_x = false, mirror_y = false, flipXY = false;
        coord_t center_x = 0, center_y = 0;
        
        // Portrait orientation will make sure the drawed polygons are rotated
        // by 90 degrees.
        Trafo(Orientation o = roLandscape, const TMirroring &mirror = NoMirror)
            // XY flipping implicitly does an X mirror
            : mirror_x(o == roPortrait ? !mirror[0] : mirror[0])
            , mirror_y(!mirror[1]) // Makes raster origin to be top left corner
            , flipXY(o == roPortrait)
        {}
        
        TMirroring get_mirror() const { return { (roPortrait ? !mirror_x : mirror_x), mirror_y}; }
        Orientation get_orientation() const { return flipXY ? roPortrait : roLandscape; }
        Point get_center() const { return {center_x, center_y}; }
    };
    
    virtual ~RasterBase() = default;
    
    /// Draw a polygon with holes.
    virtual void draw(const ExPolygon& poly) = 0;

    /// Draw a polygon with holes in binary (threshold) mode, bypassing AA.
    /// Default falls back to draw(): vector rasters (e.g. SVG) have no AA to
    /// exempt, so inheriting the default is semantically correct.
    virtual void draw_binary(const ExPolygon& poly) { draw(poly); }

    /// Apply an in-place post-process (e.g. blur / gray quantization) over the
    /// pixel buffer. Default is a no-op: rasters without a pixel buffer (e.g.
    /// SVG) correctly inherit no behavior.
    virtual void apply_postprocess() {}

    /// Return the raster to the state a freshly constructed one would be in, so
    /// that one instance can serve consecutive layers instead of being rebuilt
    /// for each of them.
    ///
    /// Deliberately has no default. Unlike draw_binary() and apply_postprocess()
    /// above, "do nothing" is not a safe fallback here: an implementation that
    /// forgets to clear what it accumulates would emit the previous layer's
    /// content on top of the current one, and that is silently wrong output
    /// rather than a missing feature. Making it pure turns the omission into a
    /// compile error.
    virtual void reset() = 0;

    /// Get the resolution of the raster.
//    virtual Resolution resolution() const = 0;
//    virtual PixelDim   pixel_dimensions() const = 0;
    virtual Trafo      trafo() const = 0;

    /// Which tiles of the pixel buffer have been written, or nullptr when the
    /// raster offers no such record (e.g. SVG, which has no pixel buffer).
    /// nullptr means consumers must treat the whole buffer as written.
    virtual const TileMap *written_tiles() const { return nullptr; }

    /// Under SLA_RASTER_VERIFY=1, check that every clean tile really holds the
    /// background (invariant I1) and report layer_id with the offending tile.
    /// Default is a no-op for rasters without a tile map.
    virtual void verify_written_tiles(size_t layer_id) const { (void) layer_id; }

    // Test only (tasks 2.16): the same scan as verify_written_tiles() with the
    // SLA_RASTER_VERIFY gate left out. The gate reads the environment once per
    // process, so a unit test cannot turn it on and off again; this is how the
    // scan itself gets exercised. The engine never calls this.
    virtual void test_only_verify_written_tiles(size_t layer_id) const { (void) layer_id; }

    virtual EncodedRaster encode(RasterEncoder encoder) const = 0;
};

struct PNGRasterEncoder {
    EncodedRaster operator()(const void *ptr, size_t w, size_t h, size_t num_components);
};

struct PNGPreviewEncoder {
    double scale;  // e.g. 0.25 = 1/4 size
    explicit PNGPreviewEncoder(double s = 0.25) : scale(s) {}
    EncodedRaster operator()(const void *ptr, size_t w, size_t h, size_t num_components);
};

// Tile-aware preview encoder (design D6 (4)). Same PNG bytes as
// PNGPreviewEncoder: destination pixels whose source block sits in a clean tile
// are left at the background 0 they were initialized to instead of being summed
// from the buffer. Needs T % n == 0 so a block cannot straddle two tiles; when
// that or the fixed-block condition does not hold, it falls back to the dense
// path and reads every pixel.
struct SparsePNGPreviewEncoder {
    double scale;
    explicit SparsePNGPreviewEncoder(double s = 0.25) : scale(s) {}
    EncodedRaster operator()(const void *ptr, size_t w, size_t h, size_t num_components,
                             const TileMap &tiles);
};

// [layer-rle] Encode a grayscale layer to the PRZ V3.0 RLE format (row-major,
// 0x55 header, BLACK/WHITE/GRAY runs, trailing ~sum checksum). Must stay
// byte-identical to agent/prz_encoder.py:_rle_encode_layer so the backend can
// consume it directly and produce a bit-for-bit identical PRZ.
//
// Single-channel buffers on little-endian targets take a word-at-a-time run scan
// (design D2); anything else, or SLA_RASTER_FASTPATH=0, uses
// rle_encode_reference(). Both produce the same bytes.
struct RLERasterEncoder {
    EncodedRaster operator()(const void *ptr, size_t w, size_t h, size_t num_components);
};

// Encoder that also gets the raster's written tiles, so it can charge the clean
// parts of the buffer to the run length by arithmetic instead of reading them.
using SparseRasterEncoder = std::function<EncodedRaster(
    const void *ptr, size_t w, size_t h, size_t num_components, const TileMap &tiles)>;

// [layer-rle] Tile-aware RLE encoder (design D6 ③). Byte-for-byte identical to
// RLERasterEncoder: it produces the same maximal same-value runs, in the same
// row-major order, and runs still continue across row ends.
//
// Precondition, beyond those of the tile map itself: the background of the
// clean tiles is 0, because that is the value the skipped pixels are charged
// as. SL1's rasters are black-backed, which is why this is only wired up there.
// Falls back to rle_encode_reference() for anything the word scan cannot take
// (more than one component, big-endian, SLA_RASTER_FASTPATH=0).
struct SparseRLERasterEncoder {
    EncodedRaster operator()(const void *ptr, size_t w, size_t h, size_t num_components,
                             const TileMap &tiles);
};

// ---------------------------------------------------------------------------
// Reference raster scans (design D8).
//
// These are the byte-by-byte RLE walk and the block-major integer preview
// downscale exactly as they were before the fast paths, kept unmodified. They
// serve as the differential oracle for the [raster-scan] unit tests and as the
// path taken when SLA_RASTER_FASTPATH=0. Do not optimize them: their value is
// that they are the old code.
// ---------------------------------------------------------------------------

// Same contract and output as RLERasterEncoder::operator().
EncodedRaster rle_encode_reference(const void *ptr, size_t w, size_t h, size_t num_components);

// Box filter downscale by an exact 1/N over whole N x N blocks: reads source
// columns [0, new_w * n) and rows [0, new_h * n) of a buffer `w` pixels wide;
// pixels right of or below those ranges are ignored.
//
// Preconditions -- the caller must guarantee them, nothing here checks:
//   - n >= 1;
//   - new_w * n <= w and new_h * n <= h, where h is the source height in pixels.
//     Breaking the width condition reads wrong pixels (a block spills into the
//     next row), and on the last rows it also reads past the source buffer.
//     Breaking the height condition reads past the source buffer outright;
//   - src holds w * h * num_components bytes and dst holds
//     new_w * new_h * num_components bytes;
//   - n * n * 255 fits in unsigned, so a block sum cannot wrap.
void preview_box_downscale_integer_reference(const uint8_t *src, size_t w,
                                             uint8_t *dst, size_t new_w, size_t new_h,
                                             size_t num_components, size_t n);

// For tests only (tasks 2.27): the single-channel integer downscale fast path on
// a raw buffer, i.e. the pixels PNGPreviewEncoder hands to the PNG writer, with
// the same preconditions as the reference above. Unlike the encoder it ignores
// SLA_RASTER_FASTPATH. With scalar_kernel the fixed-block kernel (design D13,
// n = 4, 5, 8, 10) adds bytes one by one -- the kernel builds without SSE2 use --
// instead of with _mm_sad_epu8, so both run on the machine running the tests.
// Any other n takes the byte-by-byte row walk either way. The engine never
// calls this.
void test_only_preview_box_downscale_integer(const uint8_t *src, size_t w,
                                             uint8_t *dst, size_t new_w, size_t new_h,
                                             size_t n, bool scalar_kernel);

// SLA_RASTER_FASTPATH switch (design D8). Returns false only when the variable is
// set to exactly "0"; unset or any other value keeps the fast paths on. The
// environment is read once per process, on first call; later changes to the
// variable have no effect.
bool raster_fastpath_enabled();

// True when SLA_RASTER_VERIFY=1: every layer is checked against its tile map
// before it is encoded (design D8). Off by default; the check reads the whole
// buffer, which is exactly the work the tile map exists to avoid.
bool raster_verify_enabled();

struct PPMRasterEncoder {
    EncodedRaster operator()(const void *ptr, size_t w, size_t h, size_t num_components);
};

std::ostream& operator<<(std::ostream &stream, const EncodedRaster &bytes);

// If gamma is zero, thresholding will be performed which disables AA.
// zero_preserving_pixel_local declares pp as RasterPostProcess describes; leave
// it false unless pp really has that property.
std::unique_ptr<RasterBase> create_raster_grayscale_aa(
    const Resolution        &res,
    const PixelDim          &pxdim,
    double                   gamma = 1.0,
    const RasterBase::Trafo &tr    = {},
    RasterPostProcessor      pp    = {},
    bool                     zero_preserving_pixel_local = false);

}} // namespace Slic3r::sla

#endif // SLARASTERBASE_HPP

///|/ Copyright (c) Prusa Research 2020 - 2022 Tomáš Mészáros @tamasmeszaros, Vojtěch Bubník @bubnikv
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef AGGRASTER_HPP
#define AGGRASTER_HPP

#include <libslic3r/SLA/RasterBase.hpp>
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Exception.hpp"

#include <cstdio>

// For rasterizing
#include <agg/agg_basics.h>
#include <agg/agg_rendering_buffer.h>
#include <agg/agg_pixfmt_gray.h>
#include <agg/agg_pixfmt_rgb.h>
#include <agg/agg_renderer_base.h>
#include <agg/agg_renderer_scanline.h>

#include <agg/agg_scanline_p.h>
#include <agg/agg_rasterizer_scanline_aa.h>
#include <agg/agg_path_storage.h>

namespace Slic3r {

inline const Polygon& contour(const ExPolygon& p) { return p.contour; }
inline const Polygons& holes(const ExPolygon& p) { return p.holes; }

namespace sla {

template<class Color> struct Colors {
    static const Color White;
    static const Color Black;
};

template<class Color> const Color Colors<Color>::White = Color{255};
template<class Color> const Color Colors<Color>::Black = Color{0};

// Pixel format adapter that marks the written tiles before forwarding each
// write to the real pixel format. agg::renderer_base clips every span to the
// buffer before calling down, so the coordinates here are always inside it.
// Both draw() and draw_binary() go through _draw() -> renderer_base, so the
// model and support passes are marked alike, and clear() is marked too.
//
// Only the methods this pipeline calls are provided: width/height (renderer_base
// constructor and clipping), copy_hline (clear), blend_hline (solid interior)
// and blend_solid_hspan (AA edges). Member functions of a class template are
// only instantiated when called, so a future write through any other method
// (copy_pixel, copy_bar, ...) fails to compile instead of silently writing a
// pixel without marking its tile.
template<class PixFmt> class TileTrackingPixfmt {
public:
    using color_type = typename PixFmt::color_type;
    using row_data   = typename PixFmt::row_data;

    TileTrackingPixfmt(PixFmt &pixfmt, TileMap &tiles)
        : m_pixfmt(&pixfmt), m_tiles(&tiles)
    {}

    unsigned width() const { return m_pixfmt->width(); }
    unsigned height() const { return m_pixfmt->height(); }

    void copy_hline(int x, int y, unsigned len, const color_type &c)
    {
        mark(x, y, len);
        m_pixfmt->copy_hline(x, y, len, c);
    }

    void blend_hline(int x, int y, unsigned len, const color_type &c, agg::int8u cover)
    {
        mark(x, y, len);
        m_pixfmt->blend_hline(x, y, len, c, cover);
    }

    void blend_solid_hspan(int x, int y, unsigned len, const color_type &c, const agg::int8u *covers)
    {
        mark(x, y, len);
        m_pixfmt->blend_solid_hspan(x, y, len, c, covers);
    }

private:
    // Marks even when nothing ends up changing (zero cover, transparent
    // color): marking more than was written is allowed, missing a write is not.
    void mark(int x, int y, unsigned len)
    {
        assert(x >= 0 && y >= 0);
        m_tiles->mark_span(size_t(x), size_t(y), len);
    }

    PixFmt  *m_pixfmt;
    TileMap *m_tiles;
};

template<class PixelRenderer,
         template<class /*agg::renderer_base<PixelRenderer>*/> class Renderer,
         class Rasterizer = agg::rasterizer_scanline_aa<>,
         class Scanline   = agg::scanline_p8>
class AGGRaster: public RasterBase {
public:
    using TColor = typename PixelRenderer::color_type;
    using TValue = typename TColor::value_type;
    using TPixel = typename PixelRenderer::pixel_type;
    using TRawBuffer = agg::rendering_buffer;
    using TTrackingPixfmt = TileTrackingPixfmt<PixelRenderer>;

protected:

    Resolution m_resolution;
    PixelDim m_pxdim_scaled;    // used for scaled coordinate polygons

    std::vector<TPixel> m_buf;
    agg::rendering_buffer m_rbuf;

    PixelRenderer m_pixrenderer;

    // Written-tile record for m_buf, marked by m_tracking_pixfmt on every
    // write that goes through m_raw_renderer, marked whole before a
    // whole-buffer post-process, and cleared by reset() once the buffer is
    // background again.
    TileMap m_tiles;
    TTrackingPixfmt m_tracking_pixfmt;

    agg::renderer_base<TTrackingPixfmt> m_raw_renderer;
    Renderer<agg::renderer_base<TTrackingPixfmt>> m_renderer;
    
    Trafo m_trafo;
    Scanline m_scanlines;
    Rasterizer m_rasterizer;

    // Injected format-specific in-place post-process (e.g. SL1 blur/quant).
    // Empty by default -> apply_postprocess() is a no-op.
    RasterPostProcess m_postproc;
    // m_postproc may run over the written tiles only: it is declared
    // zero-preserving and pixel-local, and the background really is zero.
    bool m_postproc_sparse = false;
    // Defensive copy of the construction-time gamma so draw_binary() can
    // restore it after temporarily switching to a threshold LUT.
    std::function<double(double)> m_gammafn;
    // Background the buffer was cleared to at construction; reset() needs it to
    // put a reused instance back into its as-constructed state.
    TColor m_background;

    // Fill len pixels with px. One-byte pixels (gray8, the only instantiation
    // in use) become a memset; anything wider falls back to a per-pixel copy.
    static void fill_pixels(TPixel *dst, size_t len, const TPixel &px)
    {
        if (sizeof(TPixel) == 1) {
            uint8_t byte;
            std::memcpy(&byte, &px, 1);
            std::memset(dst, byte, len);
        } else {
            std::fill_n(dst, len, px);
        }
    }

    void flipy(agg::path_storage &path) const
    {
        path.flip_y(0, double(m_resolution.height_px));
    }
    
    void flipx(agg::path_storage &path) const
    {
        path.flip_x(0, double(m_resolution.width_px));
    }
    
    double getPx(const Point &p) { return p(0) * m_pxdim_scaled.w_mm; }
    double getPy(const Point &p) { return p(1) * m_pxdim_scaled.h_mm; }
    agg::path_storage to_path(const Polygon &poly) { return to_path(poly.points); }
    
    template<class PointVec> agg::path_storage _to_path(const PointVec& v)
    {
        agg::path_storage path;
        
        auto it = v.begin();
        path.move_to(getPx(*it), getPy(*it));
        while(++it != v.end()) path.line_to(getPx(*it), getPy(*it));
        path.line_to(getPx(v.front()), getPy(v.front()));
        
        return path;
    }
    
    template<class PointVec> agg::path_storage _to_path_flpxy(const PointVec& v)
    {
        agg::path_storage path;
        
        auto it = v.begin();
        path.move_to(getPy(*it), getPx(*it));
        while(++it != v.end()) path.line_to(getPy(*it), getPx(*it));
        path.line_to(getPy(v.front()), getPx(v.front()));
        
        return path;
    }
    
    template<class PointVec> agg::path_storage to_path(const PointVec &v)
    {
        auto path = m_trafo.flipXY ? _to_path_flpxy(v) : _to_path(v);
        
        path.translate_all_paths(m_trafo.center_x * m_pxdim_scaled.w_mm,
                                 m_trafo.center_y * m_pxdim_scaled.h_mm);
        
        if(m_trafo.mirror_x) flipx(path);
        if(m_trafo.mirror_y) flipy(path);
        
        return path;
    }
    
    template<class P> void _draw(const P &poly)
    {
        m_rasterizer.reset();
        
        m_rasterizer.add_path(to_path(contour(poly)));
        for(auto& h : holes(poly)) m_rasterizer.add_path(to_path(h));
        
        agg::render_scanlines(m_rasterizer, m_scanlines, m_renderer);
    }
    
public:
    template<class GammaFn>
    AGGRaster(const Resolution &res,
              const PixelDim &  pd,
              const Trafo &     trafo,
              const TColor &    foreground,
              const TColor &    background,
              GammaFn &&        gammafn,
              RasterPostProcess postproc = {})
        : m_resolution(res)
        , m_pxdim_scaled(SCALING_FACTOR, SCALING_FACTOR)
        , m_buf(res.pixels())
        , m_rbuf(reinterpret_cast<TValue *>(m_buf.data()),
                 unsigned(res.width_px),
                 unsigned(res.height_px),
                 int(res.width_px *PixelRenderer::num_components))
        , m_pixrenderer(m_rbuf)
        , m_tiles(res.width_px, res.height_px)
        , m_tracking_pixfmt(m_pixrenderer, m_tiles)
        , m_raw_renderer(m_tracking_pixfmt)
        , m_renderer(m_raw_renderer)
        , m_trafo(trafo)
    {
        // Visual Studio compiler gives warnings about possible division by zero.
        assert(pd.w_mm != 0 && pd.h_mm != 0);
        if (pd.w_mm != 0 && pd.h_mm != 0) {
            m_pxdim_scaled.w_mm /= pd.w_mm;
            m_pxdim_scaled.h_mm /= pd.h_mm;
        }
        m_renderer.color(foreground);
        m_background = background;
        // Clearing goes through the tracking pixel format and marks every
        // tile; the buffer is all background afterwards, so unmark them.
        clear(m_background);
        m_tiles.clear();

        // Keep a type-erased copy of the gamma for defensive restore in
        // draw_binary(); store the injected post-processor (may be empty).
        m_gammafn = gammafn;
        m_postproc = std::move(postproc);
        // The flag promises fn(0) == 0, so skipping clean tiles is only sound
        // when the background they hold is zero.
        TPixel bg, zero;
        bg.set(m_background);
        std::memset(&zero, 0, sizeof(zero));
        m_postproc_sparse = m_postproc.zero_preserving_pixel_local &&
                            std::memcmp(&bg, &zero, sizeof(bg)) == 0 &&
                            raster_fastpath_enabled();

        m_rasterizer.gamma(gammafn);
    }
    
    Trafo trafo() const override { return m_trafo; }
    Resolution resolution() const { return m_resolution; }
    PixelDim   pixel_dimensions() const
    {
        return {SCALING_FACTOR / m_pxdim_scaled.w_mm,
                SCALING_FACTOR / m_pxdim_scaled.h_mm};
    }
    
    void draw(const ExPolygon &poly) override { _draw(poly); }

    // Binary (threshold) draw: temporarily swap the rasterizer LUT to a hard
    // threshold so the polygon is rendered without AA, then restore the
    // original gamma.
    //
    // The window between the two gamma() calls is only safe because a raster
    // instance is never touched by two threads at once. Rasters are reused
    // across layers (see SLAArchiveWriter::draw_layers), so that is no longer
    // guaranteed by construction the way per-layer instances used to guarantee
    // it -- it now rests on the instances being THREAD-BOUND. Anything that
    // hands the same raster to a second thread, such as a shared pool that
    // work-stealing can dip into, reintroduces the race here. This is a
    // correctness requirement, not a performance preference.
    //
    // The tile map rests on the same binding. m_tiles belongs to this
    // instance, and reset() clears exactly the tiles this instance marked, so
    // which layer the thread picks up next does not matter. A second thread
    // drawing into the same instance could write a pixel after its tile was
    // cleared, leaving a written pixel in a clean tile.
    void draw_binary(const ExPolygon &poly) override
    {
        m_rasterizer.gamma(agg::gamma_threshold(0.5));
        _draw(poly);
        if (m_gammafn) m_rasterizer.gamma(m_gammafn);
    }

    // Run the injected in-place post-process over the pixel buffer (no-op if
    // none was injected). Mutates m_buf directly (no copy).
    void apply_postprocess() override
    {
        if (!m_postproc.fn)
            return;

        if (m_postproc_sparse) {
            // Pixel-local, so row segments give the same bytes as one
            // whole-buffer call; zero-preserving, so the clean tiles it skips
            // would not have changed. Writes only inside written tiles, so the
            // map stays exact and reset() stays partial.
            const size_t width = m_resolution.width_px;
            for (size_t ty = 0; ty < m_tiles.tiles_y(); ++ty) {
                const TileMap::PixelSpan rows = m_tiles.pixel_rows(ty);
                m_tiles.for_each_written_span(ty, [&](TileMap::PixelSpan cols) {
                    const size_t len = cols.end - cols.begin;
                    for (size_t y = rows.begin; y < rows.end; ++y)
                        m_postproc.fn(m_buf.data() + y * width + cols.begin, len, 1,
                                      PixelRenderer::num_components);
                });
            }
            return;
        }

        // The post-process writes m_buf directly, bypassing the tracking pixel
        // format, and nothing says it stays inside the written tiles (SL1's
        // blur spreads brightness into clean neighbours). Mark the whole buffer
        // BEFORE running it, as I2 requires of any write: if fn throws halfway,
        // the map still covers whatever it changed.
        m_tiles.mark_all();
        m_postproc.fn(m_buf.data(),
                      m_resolution.width_px, m_resolution.height_px,
                      PixelRenderer::num_components);
    }

    // Put a reused instance back into its as-constructed state.
    void reset() override
    {
        // Only the written tiles, not the whole buffer. By I1 every clean tile
        // already holds the background, and by I2 every pixel written since
        // the last reset lies in a written tile, so filling the written spans
        // leaves the buffer byte-for-byte what clear(m_background) would. That
        // is what keeps the consumers consistent: the encoders and the
        // post-process may skip clean tiles, or walk every pixel, and see the
        // same image either way. A fully marked map degrades to a full clear.
        //
        // Writes m_buf directly, not through m_raw_renderer, which would mark
        // every tile it clears.
        if (!raster_fastpath_enabled()) {
            // SLA_RASTER_FASTPATH=0: the pre-tile-map behaviour, the whole
            // buffer every time. clear() marks every tile on the way, so the
            // map goes back to all-clean with the buffer all background.
            clear(m_background);
            m_tiles.clear();
            if (m_gammafn) m_rasterizer.gamma(m_gammafn);
            return;
        }

        const size_t width = m_resolution.width_px;
        TPixel bg;
        bg.set(m_background);
        for (size_t ty = 0; ty < m_tiles.tiles_y(); ++ty) {
            const TileMap::PixelSpan rows = m_tiles.pixel_rows(ty);
            m_tiles.for_each_written_span(ty, [&](TileMap::PixelSpan cols) {
                const size_t len = cols.end - cols.begin;
                for (size_t y = rows.begin; y < rows.end; ++y)
                    fill_pixels(m_buf.data() + y * width + cols.begin, len, bg);
            });
        }
        m_tiles.clear();
        // draw_binary() leaves the threshold LUT installed if no gamma was
        // captured to restore. Re-applying unconditionally makes the reused
        // state exact no matter how the previous layer happened to be drawn.
        if (m_gammafn) m_rasterizer.gamma(m_gammafn);
    }

    // SLA_RASTER_FASTPATH=0 hides the map, which sends draw_layers() to the
    // dense layer and preview encoders -- the whole of stage 2 switched off in
    // one place, since reset() and the post-process check the same flag.
    const TileMap *written_tiles() const override
    {
        return raster_fastpath_enabled() ? &m_tiles : nullptr;
    }

    // Walk the clean tiles and report the first pixel that is not background:
    // that pixel was written without its tile being marked, which breaks I1 and
    // would let reset() and the sparse consumers skip a written area. Also
    // reports a marked tile whose row summary is clear, which the span walk
    // would skip just the same. Reads the whole buffer, so it only runs under
    // SLA_RASTER_VERIFY=1.
    void verify_written_tiles(size_t layer_id) const override
    {
        if (!raster_verify_enabled()) return;
        test_only_verify_written_tiles(layer_id);
    }

    // The scan itself, with no environment gate, so a unit test can drive it
    // (tasks 2.16). The engine always goes in through verify_written_tiles().
    void test_only_verify_written_tiles(size_t layer_id) const override
    {
        const size_t width = m_resolution.width_px;
        TPixel bg;
        bg.set(m_background);

        for (size_t ty = 0; ty < m_tiles.tiles_y(); ++ty) {
            const TileMap::PixelSpan rows = m_tiles.pixel_rows(ty);
            for (size_t tx = 0; tx < m_tiles.tiles_x(); ++tx) {
                const bool written = m_tiles.written(tx, ty);
                if (written && !m_tiles.row_written(ty)) {
                    std::fprintf(stderr,
                                 "[raster-verify] layer %zu: tile (%zu, %zu) is marked "
                                 "written but its tile row summary is clear\n",
                                 layer_id, tx, ty);
                    throw Slic3r::RuntimeError("SLA_RASTER_VERIFY: inconsistent tile map");
                }
                if (written) continue;

                const TileMap::PixelSpan cols = m_tiles.pixel_columns(tx, tx + 1);
                for (size_t y = rows.begin; y < rows.end; ++y)
                    for (size_t x = cols.begin; x < cols.end; ++x) {
                        const TPixel &px = m_buf[y * width + x];
                        if (std::memcmp(&px, &bg, sizeof(TPixel)) == 0) continue;

                        unsigned value = 0;
                        std::memcpy(&value, &px, 1);
                        std::fprintf(stderr,
                                     "[raster-verify] layer %zu: pixel (%zu, %zu) = %u in "
                                     "clean tile (%zu, %zu), tile pixels [%zu, %zu) x [%zu, %zu)\n",
                                     layer_id, x, y, value, tx, ty,
                                     cols.begin, cols.end, rows.begin, rows.end);
                        throw Slic3r::RuntimeError("SLA_RASTER_VERIFY: written pixel in a clean tile");
                    }
            }
        }
    }

    EncodedRaster encode(RasterEncoder encoder) const override
    {
        return encoder(m_buf.data(), m_resolution.width_px, m_resolution.height_px, 1);
    }
    
    void clear(const TColor color) { m_raw_renderer.clear(color); }
};

/*
 * Captures an anti-aliased monochrome canvas where vectorial
 * polygons can be rasterized. Fill color is always white and the background is
 * black. Contours are anti-aliased.
 * 
 * A gamma function can be specified at compile time to make it more flexible.
 */
using _RasterGrayscaleAA =
    AGGRaster<agg::pixfmt_gray8, agg::renderer_scanline_aa_solid>;

class RasterGrayscaleAA : public _RasterGrayscaleAA {
    using Base = _RasterGrayscaleAA;
    using typename Base::TColor;
    using typename Base::TValue;
public:
    template<class GammaFn>
    RasterGrayscaleAA(const Resolution        &res,
                      const PixelDim          &pd,
                      const RasterBase::Trafo &trafo,
                      GammaFn                &&fn,
                      RasterPostProcess        postproc = {})
        : Base(res,
               pd,
               trafo,
               Colors<TColor>::White,
               Colors<TColor>::Black,
               std::forward<GammaFn>(fn),
               std::move(postproc))
    {}
    
    uint8_t read_pixel(size_t col, size_t row) const
    {
        static_assert(std::is_same<TValue, uint8_t>::value, "Not grayscale pix");
        
        uint8_t px;
        Base::m_buf[row * Base::resolution().width_px + col].get(px);
        return px;
    }
    
    void clear() { Base::clear(Colors<TColor>::Black); }
};

class RasterGrayscaleAAGammaPower: public RasterGrayscaleAA {
public:
    RasterGrayscaleAAGammaPower(const Resolution        &res,
                                const PixelDim          &pd,
                                const RasterBase::Trafo &trafo,
                                double                   gamma = 1.,
                                RasterPostProcess        postproc = {})
        : RasterGrayscaleAA(res, pd, trafo, agg::gamma_power(gamma), std::move(postproc))
    {}
};

}} // namespace Slic3r::sla

#endif // AGGRASTER_HPP

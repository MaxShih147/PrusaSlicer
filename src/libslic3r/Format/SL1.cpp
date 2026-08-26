///|/ Copyright (c) Prusa Research 2020 - 2023 Tomáš Mészáros @tamasmeszaros, Oleksandra Iushchenko @YuSanka, Lukáš Matěna @lukasmatena, Vojtěch Bubník @bubnikv
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "SL1.hpp"

#include <boost/log/trivial.hpp>
#include <boost/filesystem.hpp>

#include <sstream>

#include "libslic3r/Time.hpp"
#include "libslic3r/Zipper.hpp"
#include "libslic3r/SLAPrint.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/MTUtils.hpp"
#include "libslic3r/PrintConfig.hpp"

#include "agg/agg_rendering_buffer.h"
#include "agg/agg_pixfmt_gray.h"
#include "agg/agg_blur.h"

#include "libslic3r/miniz_extension.hpp" // IWYU pragma: keep
#include <LocalesUtils.hpp>
#include "libslic3r/GCode/ThumbnailData.hpp"
#include "libslic3r/Utils/JsonUtils.hpp"

#include "SLAArchiveReader.hpp"
#include "SLAArchiveFormatRegistry.hpp"
#include "ZipperArchiveImport.hpp"

#include "libslic3r/MarchingSquares.hpp"
#include <cstdlib>  // [layer-rle] getenv
#include <algorithm> // std::min / std::fill_n in the strip-wise blur
#include <cstdint>
#include <vector>
#include "libslic3r/PNGReadWrite.hpp"
#include "libslic3r/ClipperUtils.hpp"

#include "libslic3r/SLA/RasterBase.hpp"


#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/algorithm/string.hpp>

namespace Slic3r {

using ConfMap = std::map<std::string, std::string>;

namespace {

std::string to_ini(const ConfMap &m)
{
    std::string ret;
    for (auto &param : m)
        ret += param.first + " = " + param.second + "\n";

    return ret;
}

static std::string get_key(const std::string& opt_key)
{
    static const std::set<std::string> ms_opts = {
      "delay_before_exposure"
    , "delay_after_exposure"
    , "tilt_down_offset_delay"
    , "tilt_up_offset_delay"
    , "tilt_down_delay"
    , "tilt_up_delay"
    };
    
    static const std::set<std::string> nm_opts = {
       "tower_hop_height"
    };
    
    static const std::set<std::string> speed_opts = {
      "tower_speed"
    , "tilt_down_initial_speed"
    , "tilt_down_finish_speed"
    , "tilt_up_initial_speed"
    , "tilt_up_finish_speed"
    };

    if (ms_opts.find(opt_key) != ms_opts.end())
        return opt_key + "_ms";

    if (nm_opts.find(opt_key) != nm_opts.end())
        return opt_key + "_nm";

    if (speed_opts.find(opt_key) != speed_opts.end())
        return boost::replace_all_copy(opt_key, "_speed", "_profile");

    return opt_key;
}

namespace pt = boost::property_tree;

std::string to_json(const SLAPrint& print, const ConfMap &m)
{
    auto& cfg = print.full_print_config();

    pt::ptree below_node;
    pt::ptree above_node;

    const t_config_enum_names& tilt_enum_names  = ConfigOptionEnum< TiltSpeeds>::get_enum_names();
    const t_config_enum_names& tower_enum_names = ConfigOptionEnum<TowerSpeeds>::get_enum_names();

    for (const std::string& opt_key : tilt_options()) {
        const ConfigOption* opt = cfg.option(opt_key);
        assert(opt != nullptr);

        switch (opt->type()) {
        case coFloats: {
            auto values = static_cast<const ConfigOptionFloats*>(opt);
            double koef = opt_key == "tower_hop_height" ? 1000000. : 1000.; // export in nm (instead of mm), resp. in ms (instead of s)
            below_node.put<double>(get_key(opt_key), int(koef * values->get_at(0)));
            above_node.put<double>(get_key(opt_key), int(koef * values->get_at(1)));
        }
        break;
        case coInts: {
            auto values = static_cast<const ConfigOptionInts*>(opt);
            below_node.put<int>(get_key(opt_key), values->get_at(0));
            above_node.put<int>(get_key(opt_key), values->get_at(1));
        }
        break;
        case coBools: {
            auto values = static_cast<const ConfigOptionBools*>(opt);
            below_node.put<bool>(get_key(opt_key), values->get_at(0));
            above_node.put<bool>(get_key(opt_key), values->get_at(1));
        }
        break;
        case coEnums: {
            const t_config_enum_names& enum_names = opt_key == "tower_speed" ? tower_enum_names : tilt_enum_names;
            auto values = static_cast<const ConfigOptionEnums<TiltSpeeds>*>(opt);
            below_node.put(get_key(opt_key), enum_names[values->get_at(0)]);
            above_node.put(get_key(opt_key), enum_names[values->get_at(1)]);
        }
        break;
        case coNone:
        default:
            break;
        }
    }

    pt::ptree profile_node;
    profile_node.put("area_fill", cfg.option("area_fill")->serialize());
    profile_node.add_child("below_area_fill", below_node);
    profile_node.add_child("above_area_fill", above_node);

    pt::ptree root;
    // params from config.ini
    for (auto& param : m)
        root.put(param.first, param.second );

    root.put("version", "1");
    root.add_child("exposure_profile", profile_node);

    // Boost confirms its implementation has no 100% conformance to JSON standard. 
    // In the boost libraries, boost will always serialize each value as string and parse all values to a string equivalent.
    // so, post-prosess output
    return write_json_with_post_process(root);
}

std::string get_cfg_value(const DynamicPrintConfig &cfg, const std::string &key)
{
    std::string ret;
    
    if (cfg.has(key)) {
        auto opt = cfg.option(key);
        if (opt) ret = opt->serialize();
    }
    
    return ret;    
}

void fill_iniconf(ConfMap &m, const SLAPrint &print)
{
    CNumericLocalesSetter locales_setter; // for to_string
    auto &cfg = print.full_print_config();
    m["layerHeight"]    = get_cfg_value(cfg, "layer_height");
    m["expTime"]        = get_cfg_value(cfg, "exposure_time");
    m["expTimeFirst"]   = get_cfg_value(cfg, "initial_exposure_time");
    const std::string mps = get_cfg_value(cfg, "material_print_speed");
    m["expUserProfile"] = mps == "slow" ? "1" : mps == "fast" ? "0" : "2";
    m["materialName"]   = get_cfg_value(cfg, "sla_material_settings_id");
    m["printerModel"]   = get_cfg_value(cfg, "printer_model");
    m["printerVariant"] = get_cfg_value(cfg, "printer_variant");
    m["printerProfile"] = get_cfg_value(cfg, "printer_settings_id");
    m["printProfile"]   = get_cfg_value(cfg, "sla_print_settings_id");
    m["fileCreationTimestamp"] = Utils::utc_timestamp();
    m["prusaSlicerVersion"]    = SLIC3R_BUILD_ID;
    
    SLAPrintStatistics stats = print.print_statistics();
    // Set statistics values to the printer
    
    double used_material = (stats.objects_used_material +
                            stats.support_used_material) / 1000;
    
    int num_fade = print.default_object_config().faded_layers.getInt();
    num_fade = num_fade >= 0 ? num_fade : 0;
    
    m["usedMaterial"] = std::to_string(used_material);
    m["numFade"]      = std::to_string(num_fade);
    m["numSlow"]      = std::to_string(stats.slow_layers_count);
    m["numFast"]      = std::to_string(stats.fast_layers_count);
    m["printTime"]    = std::to_string(stats.estimated_print_time);

    bool hollow_en = false;
    auto it = print.objects().begin();
    while (!hollow_en && it != print.objects().end())
        hollow_en = (*it++)->config().hollowing_enable;

    m["hollow"] = hollow_en ? "1" : "0";
    
    m["action"] = "print";
}

void fill_slicerconf(ConfMap &m, const SLAPrint &print)
{
    using namespace std::literals::string_view_literals;
    
    // Sorted list of config keys, which shall not be stored into the ini.
    static constexpr auto banned_keys = { 
		"compatible_printers"sv,
        "compatible_prints"sv,
        //FIXME The print host keys should not be exported to full_print_config anymore. The following keys may likely be removed.
        "print_host"sv,
        "printhost_apikey"sv,
        "printhost_cafile"sv
    };
    
    assert(std::is_sorted(banned_keys.begin(), banned_keys.end()));
    auto is_banned = [](const std::string &key) {
        return std::binary_search(banned_keys.begin(), banned_keys.end(), key);
    };

    auto is_tilt_param = [](const std::string& key) -> bool {
        const auto& keys = tilt_options();
        return std::find(keys.begin(), keys.end(), key) != keys.end();
    };
    
    auto &cfg = print.full_print_config();
    for (const std::string &key : cfg.keys())
        if (! is_banned(key) && !is_tilt_param(key) && ! cfg.option(key)->is_nil())
            m[key] = cfg.opt_serialize(key);
    
}

// Vertical half of agg::stack_blur_gray8, walking the image in narrow column
// strips instead of one column at a time.
//
// WHY. agg_blur.h runs the vertical pass as `for x { for y }`. On a 15120-wide
// canvas that touches one byte per cache line and then moves a whole row away, so
// every one of the 94.2 M pixels costs its own line fill: roughly 6 GB of DRAM
// traffic for a pass that only needs to read 94 MB. Processing STRIP_COLS columns
// together makes each line fill serve all the bytes in it, which is the entire
// point of this rewrite.
//
// WHAT IS NOT CHANGED. The arithmetic is copied from agg_blur.h unaltered --
// same unsigned accumulators, same stack update order, same `(sum + whalf) / wsum`
// truncating division, same border clamping. Only the order in which columns are
// visited differs, and the vertical pass is column-independent (column x reads and
// writes nothing but column x), so visiting them in strips cannot change a single
// output byte. That is what makes the bit-exactness requirement satisfiable here;
// re-deriving the filter as a convolution would not have been.
//
// `stack_ptr` and `yp` are kept as scalars rather than per-column state on
// purpose: they are seeded identically for every column and advance identically
// on every row, so they carry no column-specific information.
static void stack_blur_gray8_vertical_strips(uint8_t *buf,
                                             unsigned w,
                                             unsigned h,
                                             size_t   stride,
                                             unsigned ry)
{
    if (ry == 0 || w == 0 || h == 0) return;
    if (ry > 254) ry = 254;                       // same cap as agg_blur.h

    // One cache line wide. Wider strips would not cut traffic any further (a line
    // is already fully consumed at 64) and only enlarge the per-strip state.
    constexpr unsigned STRIP_COLS = 64;

    const unsigned hm    = h - 1;
    const unsigned div   = ry * 2 + 1;
    const unsigned wsum  = (ry + 1) * (ry + 1);
    const unsigned whalf = wsum >> 1;

    // stack[i * STRIP_COLS + j] = stack slot i of the strip's column j, so the
    // inner loop over j stays contiguous.
    std::vector<uint8_t>  stack(size_t(div) * STRIP_COLS);
    std::vector<unsigned> sum(STRIP_COLS), sum_in(STRIP_COLS), sum_out(STRIP_COLS);

    for (unsigned x0 = 0; x0 < w; x0 += STRIP_COLS) {
        const unsigned kw = std::min(STRIP_COLS, w - x0);

        std::fill_n(sum.begin(), kw, 0u);
        std::fill_n(sum_in.begin(), kw, 0u);
        std::fill_n(sum_out.begin(), kw, 0u);

        // Left half of the stack: row 0 repeated, exactly as the scalar version
        // reads img.pixel(x, 0) once and reuses it for i = 0..ry.
        const uint8_t *row0 = buf + x0;
        for (unsigned i = 0; i <= ry; ++i) {
            uint8_t *slot = stack.data() + size_t(i) * STRIP_COLS;
            for (unsigned j = 0; j < kw; ++j) {
                const unsigned v = row0[j];
                slot[j]     = uint8_t(v);
                sum[j]     += v * (i + 1);
                sum_out[j] += v;
            }
        }

        // Right half: rows 1..ry, clamped to the last row.
        for (unsigned i = 1; i <= ry; ++i) {
            const uint8_t *row  = buf + size_t(i > hm ? hm : i) * stride + x0;
            uint8_t       *slot = stack.data() + size_t(ry + i) * STRIP_COLS;
            for (unsigned j = 0; j < kw; ++j) {
                const unsigned v = row[j];
                slot[j]    = uint8_t(v);
                sum[j]    += v * (ry + 1 - i);
                sum_in[j] += v;
            }
        }

        unsigned stack_ptr = ry;
        unsigned yp        = ry > hm ? hm : ry;
        // agg_blur.h reads img.pixel(x, yp) here; that value is overwritten before
        // any use inside the row loop below, so the read is dropped.

        for (unsigned y = 0; y < h; ++y) {
            unsigned stack_start = stack_ptr + div - ry;
            if (stack_start >= div) stack_start -= div;

            if (++yp > hm) yp = hm;

            unsigned next_ptr = stack_ptr + 1;
            if (next_ptr >= div) next_ptr = 0;

            uint8_t       *out_row  = buf + size_t(y) * stride + x0;
            const uint8_t *in_row   = buf + size_t(yp) * stride + x0;
            uint8_t       *out_slot = stack.data() + size_t(stack_start) * STRIP_COLS;
            const uint8_t *nxt_slot = stack.data() + size_t(next_ptr) * STRIP_COLS;

            for (unsigned j = 0; j < kw; ++j) {
                // Write before reading in_row: on the last row the clamp makes
                // yp == y, and the scalar version reads the byte it has just
                // written. Keeping the order preserves that behaviour.
                out_row[j] = uint8_t((sum[j] + whalf) / wsum);

                sum[j]     -= sum_out[j];
                sum_out[j] -= out_slot[j];

                const unsigned v = in_row[j];
                out_slot[j] = uint8_t(v);
                sum_in[j]  += v;
                sum[j]     += sum_in[j];

                sum_out[j] += nxt_slot[j];
                sum_in[j]  -= nxt_slot[j];
            }

            stack_ptr = next_ptr;
        }
    }
}

} // namespace

std::unique_ptr<sla::RasterBase> SL1Archive::create_raster() const
{
    sla::Resolution res;
    sla::PixelDim   pxdim;
    std::array<bool, 2>         mirror;

    double w  = m_cfg.display_width.getFloat();
    double h  = m_cfg.display_height.getFloat();
    auto   pw = size_t(m_cfg.display_pixels_x.getInt());
    auto   ph = size_t(m_cfg.display_pixels_y.getInt());

    mirror[X] = m_cfg.display_mirror_x.getBool();
    mirror[Y] = m_cfg.display_mirror_y.getBool();
    
    auto ro = m_cfg.display_orientation.getInt();
    sla::RasterBase::Orientation orientation =
        ro == sla::RasterBase::roPortrait ? sla::RasterBase::roPortrait :
                                            sla::RasterBase::roLandscape;
    
    if (orientation == sla::RasterBase::roPortrait) {
        std::swap(w, h);
        std::swap(pw, ph);
    }

    res   = sla::Resolution{pw, ph};
    pxdim = sla::PixelDim{w / pw, h / ph};
    sla::RasterBase::Trafo tr{orientation, mirror};

    double gamma = m_cfg.gamma_correction.getFloat();
    if (! m_cfg.anti_aliasing.getBool())
        gamma = 0.;

    // SL1-specific in-place post-process (AA gray quantization + blur). This is
    // the recipe that used to live in get_encoder(); it is injected here as a
    // RasterPostProcessor so the generic raster core never sees machine-specific
    // formulas. When AA is off, no post-process is injected (gamma=0 already
    // yields binary output). The dual-track pipeline runs this BEFORE drawing the
    // binary support, so support pixels are never blurred (see rasterize()).
    sla::RasterPostProcessor pp; // empty == no-op
    if (m_cfg.anti_aliasing.getBool()) {
        const int aa_level_cfg        = m_cfg.anti_aliasing_level.getInt();
        const int anti_aliasing_level = 1 << (aa_level_cfg + 1); // 0,1,2 -> 2x,4x,8x
        const int gray_level          = m_cfg.gray_level.getInt();
        const int blur_config         = m_cfg.blur.getInt();

        pp = [anti_aliasing_level, gray_level, blur_config]
             (void *ptr, size_t w, size_t h, size_t num_components)
        {
            uint8_t     *buf      = static_cast<uint8_t*>(ptr);
            const size_t bufsize  = w * h * num_components;
            const double gray_interval = 255.0 / double(anti_aliasing_level);
            const int    init_val = 32 * gray_level;

            // Per-pixel AA gray quantization (skips pure 0/255).
            auto quant = [&](uint8_t &c) {
                if (c == 0 || c == 255) return;
                if (gray_level == 8) { c = 255; return; }
                double g = (double)c;
                c = (uint8_t)((unsigned((g / gray_interval) + 0.5) / (double)anti_aliasing_level * 255.0) + 0.5);
                if (gray_level > 0)
                    c = (uint8_t)((double((256 - init_val) * ((unsigned)c + 1)) / 256.0 + init_val - 1) + 0.5);
            };

            // Fast-skip empty/full 8-byte chunks.
            size_t i = 0;
            const size_t aligned = bufsize - (bufsize % 8);
            for (; i < aligned; i += 8) {
                uint64_t *chunk = reinterpret_cast<uint64_t*>(&buf[i]);
                if (*chunk == 0 || *chunk == 0xFFFFFFFFFFFFFFFFULL) continue;
                for (size_t j = 0; j < 8; ++j) quant(buf[i + j]);
            }
            for (; i < bufsize; ++i) quant(buf[i]);

            // Blur via AGG stack blur, alpha-blended back toward the sharp
            // original (k: blur=1 -> 0.6/154, blur=2 -> 0.8/205, blur>=3 -> 1.0/256).
            // The two passes are issued separately: agg::stack_blur_gray8 keeps the
            // horizontal one (it already walks rows and is cache-friendly), and the
            // vertical one goes through the strip version, which is the same
            // arithmetic in a memory order that does not thrash the cache. Passing
            // ry = 0 makes AGG skip its own vertical pass.
            if (blur_config > 0) {
                const unsigned radius = static_cast<unsigned>(blur_config);
                const int k = (blur_config == 1) ? 154 : (blur_config == 2) ? 205 : 256;
                const size_t stride = w * num_components;
                if (k >= 256) {
                    agg::rendering_buffer rbuf(buf, (unsigned)w, (unsigned)h, (int)stride);
                    agg::pixfmt_gray8 pixf(rbuf);
                    agg::stack_blur_gray8(pixf, radius, 0);
                    stack_blur_gray8_vertical_strips(buf, (unsigned)w, (unsigned)h, stride, radius);
                } else {
                    std::vector<uint8_t> temp_buf(buf, buf + bufsize);
                    agg::rendering_buffer rbuf(temp_buf.data(), (unsigned)w, (unsigned)h, (int)stride);
                    agg::pixfmt_gray8 pixf(rbuf);
                    agg::stack_blur_gray8(pixf, radius, 0);
                    stack_blur_gray8_vertical_strips(temp_buf.data(), (unsigned)w, (unsigned)h, stride, radius);
                    for (size_t p = 0; p < bufsize; ++p)
                        buf[p] = (uint8_t)((buf[p] * (256 - k) + temp_buf[p] * k) >> 8);
                }
            }
        };
    }

    return sla::create_raster_grayscale_aa(res, pxdim, gamma, tr, std::move(pp));
}

sla::RasterEncoder SL1Archive::get_encoder() const
{
    // [layer-rle] Emit layers directly as PRZ-compatible RLE when requested
    // (PRZ fast path: the backend reads them verbatim, no PNG round-trip).
    // Opt-in via SLA_LAYER_RLE; the default PNG path is untouched. Any AA/blur
    // post-processing already ran in the raster (see create_raster), so the
    // pixel buffer is final regardless of which encoder we pick.
    if (std::getenv("SLA_LAYER_RLE"))
        return sla::RLERasterEncoder{};

    // Post-processing (AA quantization + blur) now runs in the raster via the
    // injected RasterPostProcessor (see create_raster), applied before the binary
    // support is drawn. The encoder is therefore a plain PNG writer.
    return sla::PNGRasterEncoder{};
}

static void write_thumbnail(Zipper &zipper, const ThumbnailData &data)
{
    size_t png_size = 0;

    void  *png_data = tdefl_write_image_to_png_file_in_memory_ex(
         (const void *) data.pixels.data(), data.width, data.height, 4,
         &png_size, MZ_DEFAULT_LEVEL, 1);

    if (png_data != nullptr) {
        zipper.add_entry("thumbnail/thumbnail" + std::to_string(data.width) +
                             "x" + std::to_string(data.height) + ".png",
                         static_cast<const std::uint8_t *>(png_data),
                         png_size);

        mz_free(png_data);
    }
}

void SL1Archive::export_print(Zipper               &zipper,
                              const SLAPrint       &print,
                              const ThumbnailsList &thumbnails,
                              const std::string    &prjname)
{
    std::string project =
        prjname.empty() ?
            boost::filesystem::path(zipper.get_filename()).stem().string() :
            prjname;

    ConfMap iniconf, slicerconf;
    fill_iniconf(iniconf, print);

    iniconf["jobDir"] = project;

    fill_slicerconf(slicerconf, print);

    try {
        zipper.add_entry("config.ini");
        zipper << to_ini(iniconf);
        zipper.add_entry("prusaslicer.ini");
        zipper << to_ini(slicerconf);

        zipper.add_entry("config.json");
        zipper << to_json(print, iniconf);

        size_t i = 0;
        for (const sla::EncodedRaster &rst : m_layers) {

            std::string imgname = project + string_printf("%.5d", i++) + "." +
                                  rst.extension();

            zipper.add_entry(imgname.c_str(), rst.data(), rst.size());
        }

        for (const ThumbnailData& data : thumbnails)
            if (data.is_valid())
                write_thumbnail(zipper, data);

        zipper.finalize();
    } catch(std::exception& e) {
        BOOST_LOG_TRIVIAL(error) << e.what();
        // Rethrow the exception
        throw;
    }
}

void SL1Archive::export_print(const std::string     fname,
                              const SLAPrint       &print,
                              const ThumbnailsList &thumbnails,
                              const std::string    &prjname)
{
    Zipper zipper{fname, Zipper::FAST_COMPRESSION};

    export_print(zipper, print, thumbnails, prjname);
}

} // namespace Slic3r

// /////////////////////////////////////////////////////////////////////////////
// Reader implementation
// /////////////////////////////////////////////////////////////////////////////

namespace marchsq {

template<> struct _RasterTraits<Slic3r::png::ImageGreyscale> {
    using Rst = Slic3r::png::ImageGreyscale;

       // The type of pixel cell in the raster
    using ValueType = uint8_t;

       // Value at a given position
    static uint8_t get(const Rst &rst, size_t row, size_t col)
    {
        return rst.get(row, col);
    }

       // Number of rows and cols of the raster
    static size_t rows(const Rst &rst) { return rst.rows; }
    static size_t cols(const Rst &rst) { return rst.cols; }
};

} // namespace marchsq

namespace Slic3r {

template<class Fn> static void foreach_vertex(ExPolygon &poly, Fn &&fn)
{
    for (auto &p : poly.contour.points) fn(p);
    for (auto &h : poly.holes)
        for (auto &p : h.points) fn(p);
}

void invert_raster_trafo(ExPolygons &                  expolys,
                         const sla::RasterBase::Trafo &trafo,
                         coord_t                       width,
                         coord_t                       height)
{
    if (trafo.flipXY) std::swap(height, width);

    for (auto &expoly : expolys) {
        if (trafo.mirror_y)
            foreach_vertex(expoly, [height](Point &p) {p.y() = height - p.y(); });

        if (trafo.mirror_x)
            foreach_vertex(expoly, [width](Point &p) {p.x() = width - p.x(); });

        expoly.translate(-trafo.center_x, -trafo.center_y);

        if (trafo.flipXY)
            foreach_vertex(expoly, [](Point &p) { std::swap(p.x(), p.y()); });

        if ((trafo.mirror_x + trafo.mirror_y + trafo.flipXY) % 2) {
            expoly.contour.reverse();
            for (auto &h : expoly.holes) h.reverse();
        }
    }
}

RasterParams get_raster_params(const DynamicPrintConfig &cfg)
{
    auto *opt_disp_cols = cfg.option<ConfigOptionInt>("display_pixels_x");
    auto *opt_disp_rows = cfg.option<ConfigOptionInt>("display_pixels_y");
    auto *opt_disp_w    = cfg.option<ConfigOptionFloat>("display_width");
    auto *opt_disp_h    = cfg.option<ConfigOptionFloat>("display_height");
    auto *opt_mirror_x  = cfg.option<ConfigOptionBool>("display_mirror_x");
    auto *opt_mirror_y  = cfg.option<ConfigOptionBool>("display_mirror_y");
    auto *opt_orient    = cfg.option<ConfigOptionEnum<SLADisplayOrientation>>("display_orientation");

    if (!opt_disp_cols || !opt_disp_rows || !opt_disp_w || !opt_disp_h ||
        !opt_mirror_x || !opt_mirror_y || !opt_orient)
        throw MissingProfileError("Invalid SL1 / SL1S file");

    RasterParams rstp;

    rstp.px_w = opt_disp_w->value / (opt_disp_cols->value - 1);
    rstp.px_h = opt_disp_h->value / (opt_disp_rows->value - 1);

    rstp.trafo = sla::RasterBase::Trafo{opt_orient->value == sladoLandscape ?
                                            sla::RasterBase::roLandscape :
                                            sla::RasterBase::roPortrait,
                                        {opt_mirror_x->value, opt_mirror_y->value}};

    rstp.height = scaled(opt_disp_h->value);
    rstp.width  = scaled(opt_disp_w->value);

    return rstp;
}

namespace {

ExPolygons rings_to_expolygons(const std::vector<marchsq::Ring> &rings,
                               double px_w, double px_h)
{
    auto polys = reserve_vector<ExPolygon>(rings.size());

    for (const marchsq::Ring &ring : rings) {
        Polygon poly; Points &pts = poly.points;
        pts.reserve(ring.size());

        for (const marchsq::Coord &crd : ring)
            pts.emplace_back(scaled(crd.c * px_w), scaled(crd.r * px_h));

        polys.emplace_back(poly);
    }

    // TODO: Is a union necessary?
    return union_ex(polys);
}

std::vector<ExPolygons> extract_slices_from_sla_archive(
    ZipperArchive           &arch,
    const RasterParams      &rstp,
    const marchsq::Coord    &win,
    std::function<bool(int)> progr)
{
    std::vector<ExPolygons> slices(arch.entries.size());

    struct Status
    {
        double                                 incr, val, prev;
        bool                                   stop  = false;
        execution::SpinningMutex<ExecutionTBB> mutex = {};
    } st{100. / slices.size(), 0., 0.};

    execution::for_each(
        ex_tbb, size_t(0), arch.entries.size(),
        [&arch, &slices, &st, &rstp, &win, progr](size_t i) {
            // Status indication guarded with the spinlock
            {
                std::lock_guard lck(st.mutex);
                if (st.stop) return;

                st.val += st.incr;
                double curr = std::round(st.val);
                if (curr > st.prev) {
                    st.prev = curr;
                    st.stop = !progr(int(curr));
                }
            }

            png::ImageGreyscale img;
            png::ReadBuf        rb{arch.entries[i].buf.data(),
                            arch.entries[i].buf.size()};
            if (!png::decode_png(rb, img)) return;

            constexpr uint8_t isoval = 128;
            auto              rings = marchsq::execute(img, isoval, win);
            ExPolygons        expolys = rings_to_expolygons(rings, rstp.px_w,
                                                            rstp.px_h);

            // Invert the raster transformations indicated in the profile metadata
            invert_raster_trafo(expolys, rstp.trafo, rstp.width, rstp.height);

            slices[i] = std::move(expolys);
        },
        execution::max_concurrency(ex_tbb));

    if (st.stop) slices = {};

    return slices;
}

} // namespace

ConfigSubstitutions SL1Reader::read(std::vector<ExPolygons> &slices,
                                    DynamicPrintConfig      &profile_out)
{
    Vec2i windowsize;

    switch(m_quality)
    {
    case SLAImportQuality::Fast: windowsize = {8, 8}; break;
    case SLAImportQuality::Balanced: windowsize = {4, 4}; break;
    default:
    case SLAImportQuality::Accurate:
        windowsize = {2, 2}; break;
    };

    // Ensure minimum window size for marching squares
    windowsize.x() = std::max(2, windowsize.x());
    windowsize.y() = std::max(2, windowsize.y());

    std::vector<std::string> includes = { "ini", "png"};
    std::vector<std::string> excludes = { "thumbnail" };
    ZipperArchive arch = read_zipper_archive(m_fname, includes, excludes);
    auto [profile_use, config_substitutions] = extract_profile(arch, profile_out);

    RasterParams   rstp = get_raster_params(profile_use);
    marchsq::Coord win  = {windowsize.y(), windowsize.x()};
    slices = extract_slices_from_sla_archive(arch, rstp, win, m_progr);

    return std::move(config_substitutions);
}

ConfigSubstitutions SL1Reader::read(DynamicPrintConfig &out)
{
    ZipperArchive arch = read_zipper_archive(m_fname, {"ini"}, {"png", "thumbnail"});
    return out.load(arch.profile, ForwardCompatibilitySubstitutionRule::Enable);
}

} // namespace Slic3r

#include <cstdio>
#include <cstdint>
#include <string>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <unordered_set>
#include <math.h>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <boost/nowide/args.hpp>
#include <boost/nowide/cstdlib.hpp>
#include <boost/nowide/iostream.hpp>
#include <boost/nowide/filesystem.hpp>
#include <boost/nowide/cstdio.hpp>
#include <boost/nowide/iostream.hpp>
#include <boost/nowide/fstream.hpp>
#include <boost/dll/runtime_symbol_info.hpp>

#include "libslic3r/libslic3r.h"
#if !SLIC3R_OPENGL_ES
#include <boost/algorithm/string/split.hpp>
#endif // !SLIC3R_OPENGL_ES
#include "libslic3r/Config.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/GCode/PostProcessor.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Preset.hpp"
#include <arrange-wrapper/ModelArrange.hpp>
#include "libslic3r/Print.hpp"
#include "libslic3r/SLAPrint.hpp"
#include "libslic3r/Format/AMF.hpp"
#include "libslic3r/Format/3mf.hpp"
#include "libslic3r/Format/STL.hpp"
#include "libslic3r/Format/OBJ.hpp"
#include "libslic3r/Format/SL1.hpp"
#include "libslic3r/miniz_extension.hpp"
#include "libslic3r/PNGReadWrite.hpp"
#include "libslic3r/MultipleBeds.hpp"
#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/SLA/Hollowing.hpp"
#include "libslic3r/SLA/ModelFingerprint.hpp"
#include "libslic3r/SLA/SupportPointIO.hpp"
#include "libslic3r/SLA/PriorPillarIO.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include "CLI/CLI.hpp"
#include "CLI/ProfilesSharingUtils.hpp"

#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include "stb_image_resize2.h"

namespace Slic3r::CLI {

static bool has_profile_sharing_action(const Data& cli)
{
    return cli.actions_config.has("query-printer-models") || cli.actions_config.has("query-print-filament-profiles");
}

bool has_full_config_from_profiles(const Data& cli)
{
    const DynamicPrintConfig& input = cli.input_config;
    return  !has_profile_sharing_action(cli) &&
           (input.has("print-profile") && !input.opt_string("print-profile").empty() ||
            input.has("material-profile") && !input.option<ConfigOptionStrings>("material-profile")->values.empty() ||
            input.has("printer-profile") && !input.opt_string("printer-profile").empty());
}

bool process_profiles_sharing(const Data& cli)
{
    if (!has_profile_sharing_action(cli))
        return false;

    std::string ret;

    if (cli.actions_config.has("query-printer-models")) {
        ret = Slic3r::get_json_printer_models(get_printer_technology(cli.overrides_config));
    }
    else if (cli.actions_config.has("query-print-filament-profiles")) {
        if (cli.input_config.has("printer-profile") && !cli.input_config.opt_string("printer-profile").empty()) {
            const std::string printer_profile = cli.input_config.opt_string("printer-profile");
            ret = Slic3r::get_json_print_filament_profiles(printer_profile);
            if (ret.empty()) {
                boost::nowide::cerr << "query-print-filament-profiles error: Printer profile '" << printer_profile <<
                                        "' wasn't found among installed printers." << std::endl <<
                                        "Or the request can be wrong." << std::endl;
                return true;
            }
        }
        else {
            boost::nowide::cerr << "query-print-filament-profiles error: This action requires set 'printer-profile' option" << std::endl;
            return true;
        }
    }

    if (ret.empty()) {
        boost::nowide::cerr << "Wrong request" << std::endl;
        return true;
    }

    // use --output when available

    if (cli.misc_config.has("output")) {
        std::string cmdline_param = cli.misc_config.opt_string("output");
        // if we were supplied a directory, use it and append our automatically generated filename
        boost::filesystem::path cmdline_path(cmdline_param);
        boost::filesystem::path proposed_path = boost::filesystem::path(Slic3r::resources_dir()) / "out.json";
        if (boost::filesystem::is_directory(cmdline_path))
            proposed_path = (cmdline_path / proposed_path.filename());
        else if (cmdline_path.extension().empty())
            proposed_path = cmdline_path.replace_extension("json");
        else
            proposed_path = cmdline_path;
        const std::string file = proposed_path.string();

        boost::nowide::ofstream c;
        c.open(file, std::ios::out | std::ios::trunc);
        c << ret << std::endl;
        c.close();

        boost::nowide::cout << "Output for your request is written into " << file << std::endl;
    }
    else 
        printf("%s", ret.c_str());

    return true;
}

namespace IO {
    enum ExportFormat : int {
        OBJ,
        STL,
        // SVG, 
        TMF,
        Gcode
    };
}

static std::string output_filepath(const Model& model, IO::ExportFormat format, const std::string& cmdline_param)
{
    std::string ext;
    switch (format) {
    case IO::OBJ: ext = ".obj"; break;
    case IO::STL: ext = ".stl"; break;
    case IO::TMF: ext = ".3mf"; break;
    default: assert(false); break;
    };
    auto proposed_path = boost::filesystem::path(model.propose_export_file_name_and_path(ext));
    // use --output when available
    if (!cmdline_param.empty()) {
        // if we were supplied a directory, use it and append our automatically generated filename
        boost::filesystem::path cmdline_path(cmdline_param);
        if (boost::filesystem::is_directory(cmdline_path))
            proposed_path = cmdline_path / proposed_path.filename();
        else
            proposed_path = cmdline_path;
    }
    return proposed_path.string();
}

static bool export_models(std::vector<Model>& models, IO::ExportFormat format, const std::string& cmdline_param)
{
    for (Model& model : models) {
        const std::string path = output_filepath(model, format, cmdline_param);
        bool success = false;
        switch (format) {
        case IO::OBJ: success = Slic3r::store_obj(path.c_str(), &model);          break;
        case IO::STL: success = Slic3r::store_stl(path.c_str(), &model, true);    break;
        case IO::TMF: success = Slic3r::store_3mf(path.c_str(), &model, nullptr, false); break;
        default: assert(false); break;
        }
        if (success)
            std::cout << "File exported to " << path << std::endl;
        else {
            std::cerr << "File export to " << path << " failed" << std::endl;
            return false;
        }
    }
    return true;
}


static ThumbnailData resize_and_crop(const std::vector<unsigned char>& data, int width, int height, int width_new, int height_new) {
    ThumbnailData th;

    float scale_x = float(width_new) / width;
    float scale_y = float(height_new) / height;
    float scale = std::max(scale_x, scale_y);  // Choose the larger scale to fill the box
    int resized_width = int(width * scale);
    int resized_height = int(height * scale);

    std::vector<unsigned char> resized_rgba(resized_width * resized_height * 4);
    stbir_resize_uint8_linear(data.data(), width, height, 4 * width,
                              resized_rgba.data(), resized_width, resized_height, 4 * resized_width,
                              STBIR_RGBA);

    th.set(width_new, height_new);
    int crop_x = (resized_width - width_new) / 2;
    int crop_y = (resized_height - height_new) / 2;

    for (int y = 0; y < height_new; ++y) {
        std::memcpy(th.pixels.data() + y * width_new * 4, 
                    resized_rgba.data() + ((y + crop_y) * resized_width + crop_x) * 4, 
                    width_new * 4);
    }
    return th;
}


static std::function<ThumbnailsList(const ThumbnailsParams&)> get_thumbnail_generator_cli(const std::string& filename)
{
    if (boost::iends_with(filename, ".3mf")) {
        return [filename](const ThumbnailsParams& params) {
            ThumbnailsList list_out;

            mz_zip_archive archive;
            mz_zip_zero_struct(&archive);

            if (!open_zip_reader(&archive, filename))
                return list_out;
            mz_uint num_entries = mz_zip_reader_get_num_files(&archive);
            mz_zip_archive_file_stat stat;

            int index = mz_zip_reader_locate_file(&archive, "Metadata/thumbnail.png", nullptr, 0);
            if (index < 0 || !mz_zip_reader_file_stat(&archive, index, &stat))
                return list_out;
            std::string buffer;
            buffer.resize(int(stat.m_uncomp_size));
            mz_bool res = mz_zip_reader_extract_file_to_mem(&archive, stat.m_filename, buffer.data(), (size_t)stat.m_uncomp_size, 0);
            if (res == 0)
                return list_out;
            close_zip_reader(&archive);

            std::vector<unsigned char> data;
            unsigned width = 0;
            unsigned height = 0;
            png::decode_png(buffer, data, width, height);

            {
                // Flip the image vertically so it matches the convention in Thumbnails generator.
                const int row_size = width * 4; // Each pixel is 4 bytes (RGBA)
                std::vector<unsigned char> temp_row(row_size);
                for (int i = 0; i < height / 2; ++i) {
                    unsigned char* top_row = &data[i * row_size];
                    unsigned char* bottom_row = &data[(height - i - 1) * row_size];
                    std::copy(bottom_row, bottom_row + row_size, temp_row.begin());
                    std::copy(top_row, top_row + row_size, bottom_row);
                    std::copy(temp_row.begin(), temp_row.end(), top_row);
                }
            }

            for (const Vec2d& size : params.sizes) {
                Point isize(size);
                list_out.push_back(resize_and_crop(data, width, height, isize.x(), isize.y()));
            }
            return list_out;
        };
    }

    return [](const ThumbnailsParams&) ->ThumbnailsList { return {}; };
}

static void update_instances_outside_state(Model& model, const DynamicPrintConfig& config)
{
    Pointfs bed_shape = dynamic_cast<const ConfigOptionPoints*>(config.option("bed_shape"))->values;
    BuildVolume build_volume(bed_shape, config.opt_float("max_print_height"));
    s_multiple_beds.update_build_volume(BoundingBoxf(bed_shape));
    model.update_print_volume_state(build_volume);
}

// Drop exactly duplicated triangles from a mesh imported via --import-support-stl.
// Returns the number of faces removed; the mesh is left untouched when nothing is
// duplicated (including the face order, which callers downstream rely on).
//
// THIS IS A DEFENSIVE NET, NOT THE ROOT FIX. The duplication it removes is produced
// upstream by the DS-Online support exporter, which walks the scene graph and picks
// up every mesh it finds -- including the stencil clipping passes that are attached
// as children of each support host and share the host's geometry. The same triangles
// therefore get written out five times. Only the exporter can actually fix that; all
// we can do here is stop a malformed upload from making slicing several times slower.
// The limit is worth spelling out: if the upstream ever emits near-duplicates whose
// coordinates are nudged rather than copied verbatim, nothing below will catch them
// and the bad geometry goes straight into the slicer.
//
// Duplicates are matched on the exact bits of the three vertex coordinates, in the
// order they are stored. No tolerance, no nearby merge, no winding or rotation
// canonicalisation. The asymmetry is the whole argument: a tolerant match could
// silently delete legitimate coplanar geometry -- touching support pillars, the
// raft/pillar junction -- and a missing support is a failed print, whereas leaving a
// near-duplicate in place merely costs some speed.
static size_t remove_duplicate_faces(indexed_triangle_set &its)
{
    struct FaceKey {
        uint32_t bits[9];
        bool operator==(const FaceKey &rhs) const { return std::memcmp(bits, rhs.bits, sizeof(bits)) == 0; }
    };
    struct FaceKeyHash {
        size_t operator()(const FaceKey &key) const {
            uint64_t h = 1469598103934665603ull; // FNV-1a over the raw coordinate bits
            for (uint32_t word : key.bits) { h ^= word; h *= 1099511628211ull; }
            return size_t(h);
        }
    };

    if (its.indices.size() < 2)
        return 0;

    std::unordered_set<FaceKey, FaceKeyHash> seen;
    seen.reserve(its.indices.size());
    std::vector<stl_triangle_vertex_indices> kept;
    kept.reserve(its.indices.size());

    for (const stl_triangle_vertex_indices &face : its.indices) {
        FaceKey key;
        for (int v = 0; v < 3; ++ v) {
            const stl_vertex &p = its.vertices[face(v)];
            for (int c = 0; c < 3; ++ c)
                std::memcpy(&key.bits[v * 3 + c], &p(c), sizeof(uint32_t));
        }
        if (seen.insert(key).second)
            kept.emplace_back(face);
    }

    size_t removed = its.indices.size() - kept.size();
    if (removed > 0)
        its.indices = std::move(kept);
    // Vertices are deliberately left alone: any that the dropped faces no longer
    // reference are simply unreferenced, and since every duplicate carried identical
    // coordinates the mesh bounding box is unchanged either way.
    return removed;
}

bool process_actions(Data& cli, const DynamicPrintConfig& print_config, std::vector<Model>& models)
{
    DynamicPrintConfig& actions     = cli.actions_config;
    DynamicPrintConfig& transform   = cli.transform_config;

    // Reads one CLI string option, treating an empty value as absent.
    auto opt_path = [](const DynamicPrintConfig &cfg, const char *key) -> std::string {
        return cfg.has(key) ? cfg.opt_string(key) : std::string();
    };

    const std::string import_support_points_path = opt_path(cli.misc_config, "import_support_points");
    const std::string export_support_pillars_path = opt_path(cli.misc_config, "export_support_pillars");
    const std::string export_brace_stls_dir = opt_path(cli.misc_config, "export_brace_stls");
    const std::string export_support_points_path = opt_path(actions, "export_support_points");

    // An empty path is a typo, never a way of saying "not this time". Treating
    // it as absent would leave the caller waiting for a file that was never
    // going to be written, with a successful exit code to go with it.
    if (cli.misc_config.has("import_support_points") && import_support_points_path.empty()) {
        boost::nowide::cerr << "error: --import-support-points needs a file path" << std::endl;
        return false;
    }
    if (actions.has("export_support_points") && export_support_points_path.empty()) {
        boost::nowide::cerr << "error: --export-support-points needs a file path" << std::endl;
        return false;
    }

    // The support point interface and --import-support-stl are mutually
    // exclusive. One hands the engine a finished support mesh; the other
    // describes the points the engine should build one FROM. Accepting both
    // would mean silently ignoring one of them, so this is checked here, at the
    // top of the function, before any output file exists.
    {
        const std::string stl_path = opt_path(cli.misc_config, "import_support_stl");
        const char *conflicting = nullptr;
        if (!import_support_points_path.empty())
            conflicting = "--import-support-points";
        else if (!export_support_points_path.empty())
            conflicting = "--export-support-points";

        if (!stl_path.empty() && conflicting != nullptr) {
            boost::nowide::cerr << "error: " << conflicting
                                << " cannot be combined with --import-support-stl"
                                << std::endl;
            return false;
        }
    }

    // doesn't need any aditional input 

    if (actions.has("help")) {
        print_help();
    }
    if (actions.has("help_fff")) {
        print_help(true, ptFFF);
    }
    if (actions.has("help_sla")) {
        print_help(true, ptSLA);
    }

    if (actions.has("info")) {
        if (models.empty()) {
            boost::nowide::cerr << "error: cannot show info for empty models." << std::endl;
            return 1;
        }
        // --info works on unrepaired model
        for (Model& model : models) {
            model.add_default_instances();
            model.print_info();
        }
    }

    if (actions.has("save")) {
        //FIXME check for mixing the FFF / SLA parameters.
        // or better save fff_print_config vs. sla_print_config
        print_config.save(actions.opt_string("save"));
    }

    if (models.empty() && (actions.has("export_stl") || actions.has("export_obj") || actions.has("export_3mf"))) {
        boost::nowide::cerr << "error: cannot export empty models." << std::endl;
        return 1;
    }

    const std::string output = cli.misc_config.has("output") ? cli.misc_config.opt_string("output") : "";

    if (actions.has("export_stl")) {
        for (auto& model : models)
            model.add_default_instances();
        if (!export_models(models, IO::STL, output))
            return 1;
    }
    if (actions.has("export_obj")) {
        for (auto& model : models)
            model.add_default_instances();
        if (!export_models(models, IO::OBJ, output))
            return 1;
    }
    if (actions.has("export_3mf")) {
        if (!export_models(models, IO::TMF, output))
            return 1;
    }

    if (actions.has("export_hollow_stl")) {
        if (models.empty()) {
            boost::nowide::cerr << "error: cannot hollow empty models." << std::endl;
            return 1;
        }

        for (Model& model : models) {
            model.add_default_instances();
            for (const ModelObject* mo : model.objects) {
                // Get the mesh from the first volume (simplified - assumes single volume)
                if (mo->volumes.empty()) {
                    boost::nowide::cerr << "error: model object has no volumes." << std::endl;
                    return 1;
                }

                TriangleMesh mesh = mo->volumes.front()->mesh();
                if (mesh.empty()) {
                    boost::nowide::cerr << "error: mesh is empty." << std::endl;
                    return 1;
                }

                // Get hollowing parameters from config
                sla::HollowingConfig hc;
                if (print_config.has("hollowing_min_thickness"))
                    hc.min_thickness = print_config.opt_float("hollowing_min_thickness");
                if (print_config.has("hollowing_quality"))
                    hc.quality = print_config.opt_float("hollowing_quality");
                if (print_config.has("hollowing_closing_distance"))
                    hc.closing_distance = print_config.opt_float("hollowing_closing_distance");

                boost::nowide::cout << "Generating hollow interior (thickness=" << hc.min_thickness
                                   << ", quality=" << hc.quality
                                   << ", closing=" << hc.closing_distance << ")..." << std::endl;

                // Generate the interior mesh
                auto interior = sla::generate_interior(mesh.its, hc);
                if (!interior) {
                    boost::nowide::cerr << "error: failed to generate interior mesh." << std::endl;
                    return 1;
                }

                // Get the interior mesh
                indexed_triangle_set interior_its = sla::get_mesh(*interior);
                if (interior_its.indices.empty()) {
                    boost::nowide::cerr << "error: interior mesh is empty. Try reducing wall thickness for smaller models." << std::endl;
                    return 1;
                }

                // Flip normals for proper visualization (interior faces outward in PrusaSlicer)
                sla::swap_normals(interior_its);

                // Determine output path
                std::string outpath = output;
                if (outpath.empty()) {
                    boost::filesystem::path input_path(mo->input_file.empty() ? "model" : mo->input_file);
                    outpath = input_path.stem().string() + "_hollow.stl";
                }

                // Export the interior mesh
                if (!its_write_stl_binary(outpath.c_str(), "hollow_interior", interior_its)) {
                    boost::nowide::cerr << "error: failed to write interior mesh to " << outpath << std::endl;
                    return 1;
                }

                boost::nowide::cout << "Hollow interior mesh exported to " << outpath << std::endl;
            }
        }
    }

    if (actions.has("slice") || actions.has("export_gcode") || actions.has("export_sla") || actions.has("export_support_stl") || actions.has("export_preview_pngs") || !export_support_points_path.empty()) {
        PrinterTechnology       printer_technology = Preset::printer_technology(print_config);
        if (actions.has("export_gcode") && printer_technology == ptSLA) {
            boost::nowide::cerr << "error: cannot export G-code for an FFF configuration" << std::endl;
            return 1;
        }
        else if (actions.has("export_sla") && printer_technology == ptFFF) {
            boost::nowide::cerr << "error: cannot export SLA slices for a SLA configuration" << std::endl;
            return 1;
        }
        // Support points only exist in the SLA pipeline. Without this the FFF
        // branch below would run a full slice and hand back a G-code file -
        // a completely different artifact from the one that was asked for -
        // while never writing the JSON and still exiting successfully.
        else if (!export_support_points_path.empty() && printer_technology == ptFFF) {
            boost::nowide::cerr << "error: --export-support-points requires an SLA configuration"
                                << std::endl;
            return false;
        }
        // Same reasoning in the other direction: an import that the SLA branch
        // never reaches would be silently ignored.
        else if (!import_support_points_path.empty() && printer_technology == ptFFF) {
            boost::nowide::cerr << "error: --import-support-points requires an SLA configuration"
                                << std::endl;
            return false;
        }

        // The interchange file describes ONE object, and both paths below run
        // once per input model. Left unchecked, two inputs would take turns
        // writing the same export path and only the last one would survive -
        // silently, with a successful exit code. Refused up front, before any
        // slicing happens.
        if (models.size() != 1) {
            const char *which = !export_support_points_path.empty() ? "--export-support-points"
                              : !import_support_points_path.empty() ? "--import-support-points"
                                                                    : nullptr;
            if (which != nullptr) {
                boost::nowide::cerr << "error: " << which << " handles a single input model; "
                                    << models.size() << " were given" << std::endl;
                return false;
            }
        }

        const Vec2crd           gap{ s_multiple_beds.get_bed_gap() };
        arr2::ArrangeBed        bed = arr2::to_arrange_bed(get_bed_shape(print_config), gap);
        arr2::ArrangeSettings   arrange_cfg;
        arrange_cfg.set_distance_from_objects(min_object_distance(print_config));

        for (Model& model : models) {
            // If all objects have defined instances, their relative positions will be
            // honored when printing (they will be only centered, unless --dont-arrange
            // is supplied); if any object has no instances, it will get a default one
            // and all instances will be rearranged (unless --dont-arrange is supplied).
            if (!transform.has("dont_arrange") || !transform.opt_bool("dont_arrange")) {
                if (transform.has("center")) {
                    Vec2d c = transform.option<ConfigOptionPoint>("center")->value;
                    arrange_objects(model, arr2::InfiniteBed{ scaled(c) }, arrange_cfg);
                }
                else
                    arrange_objects(model, bed, arrange_cfg);
            }

            Print       fff_print;
            SLAPrint    sla_print;
            sla_print.set_status_callback( [](const PrintBase::SlicingStatus& s) {
                if (s.percent >= 0) { // FIXME: is this sufficient?
                    printf("%3d%s %s\n", s.percent, "% =>", s.text.c_str());
                    std::fflush(stdout);
                }
            });

            PrintBase* print = (printer_technology == ptFFF) ? static_cast<PrintBase*>(&fff_print) : static_cast<PrintBase*>(&sla_print);
            if (printer_technology == ptFFF) {
                for (auto* mo : model.objects)
                    fff_print.auto_assign_extruders(mo);
            }

            // Load a caller supplied support point list (--import-support-points).
            //
            // This runs BEFORE print->apply() on purpose. apply() decides which
            // pipeline steps are invalidated from the model's contents and
            // copies that state into the print object, so points written after
            // it would sit in the model with nothing looking at them.
            // --import-support-stl below is the opposite case: it operates on an
            // SLAPrintObject, which does not exist until apply() has run.
            if (printer_technology == ptSLA && !import_support_points_path.empty()) {
                boost::nowide::ifstream ifs(import_support_points_path);
                if (!ifs.good()) {
                    boost::nowide::cerr << "error: failed to open --import-support-points: "
                                        << import_support_points_path << std::endl;
                    return false;
                }
                std::ostringstream text;
                text << ifs.rdbuf();

                // The globals the file falls back to for any size it does not
                // name. Built from the print config because no SLAPrintObject
                // exists yet at this point.
                SLAPrintObjectConfig obj_cfg;
                obj_cfg.apply(print_config, true);
                const sla::SupportTreeConfig scfg = make_support_cfg(obj_cfg);

                sla::SupportPointFile file;
                std::string parse_err;
                if (!sla::support_points_from_string(text.str(), scfg, file, parse_err)) {
                    boost::nowide::cerr << "error: --import-support-points: " << parse_err
                                        << std::endl;
                    return false;
                }

                // The interchange format carries a flat point list with no
                // object_id dimension yet (openspec task 9.8 is still open), so
                // there is no way to say which object a point belongs to.
                // Refusing beats guessing.
                if (model.objects.size() != 1) {
                    boost::nowide::cerr << "error: --import-support-points handles a single object "
                                           "per file; this model has " << model.objects.size()
                                        << std::endl;
                    return false;
                }

                ModelObject *mo = model.objects.front();

                if (file.has_fingerprint) {
                    const sla::ModelFingerprint current = sla::model_fingerprint(*mo);
                    if (!sla::fingerprint_matches(file.fingerprint, current)) {
                        // A fixed, untranslatable marker: the backend classifier
                        // matches on this line to tell "the model changed" apart
                        // from "support generation failed". Nothing is sliced and
                        // no output file is written; in particular this must NOT
                        // fall back to generating points automatically, which
                        // would silently discard the caller's edits.
                        boost::nowide::cerr << sla::support_points_model_mismatch_marker << std::endl;
                        boost::nowide::cerr << "  expected: "
                                            << sla::fingerprint_to_string(file.fingerprint) << std::endl;
                        boost::nowide::cerr << "  actual:   "
                                            << sla::fingerprint_to_string(current) << std::endl;
                        return false;
                    }
                } else {
                    // A hand written list that only adds a few points has no
                    // fingerprint to carry. Loading it is allowed, but the caller
                    // is told that nothing was verified.
                    boost::nowide::cerr << "warning: --import-support-points file carries no model "
                                           "fingerprint; loaded without checking it against the model."
                                        << std::endl;
                }

                mo->sla_support_points = file.points;
                mo->sla_points_status  = sla::PointsStatus::UserModified;
                boost::nowide::cout << "Loaded " << file.points.size()
                                    << " support points from " << import_support_points_path
                                    << std::endl;
            }

            update_instances_outside_state(model, print_config);
            MultipleBedsUtils::with_single_bed_model_fff(model, 0, [&print, &model, &print_config]()
            {
                print->apply(model, print_config);
            });

            // Import an externally generated support mesh (--import-support-stl) and
            // attach it to the single SLA print object so it is sliced as the support
            // track instead of generating supports. Must run after apply() (objects
            // exist) and before process().
            if (printer_technology == ptSLA && cli.misc_config.has("import_support_stl")) {
                const std::string support_path = cli.misc_config.opt_string("import_support_stl");
                if (!support_path.empty()) {
                    TriangleMesh support_mesh;
                    if (!support_mesh.ReadSTLFile(support_path.c_str()) || support_mesh.empty()) {
                        boost::nowide::cerr << "error: failed to read --import-support-stl: " << support_path << std::endl;
                        return 1;
                    }
                    // Strip exact duplicate faces before the mesh reaches the slicer, so that
                    // slice_supports() and merge_slices_and_eval_stats() both work on a single
                    // copy instead of N. See remove_duplicate_faces() for why this is a
                    // defensive net and not a fix for the duplication itself.
                    const size_t faces_before = support_mesh.its.indices.size();
                    if (remove_duplicate_faces(support_mesh.its) > 0) {
                        const size_t faces_after = support_mesh.its.indices.size();
                        // Formatted apart so the fixed/precision flags do not stick to cerr.
                        std::ostringstream ratio;
                        ratio << std::fixed << std::setprecision(2) << double(faces_before) / double(faces_after);
                        boost::nowide::cerr << "warning: --import-support-stl contained duplicate faces: "
                                            << faces_before << " -> " << faces_after << " (" << ratio.str()
                                            << "x). Deduplicated for slicing; the upstream exporter still needs fixing."
                                            << std::endl;
                    }
                    if (!sla_print.attach_imported_support(support_mesh.its))
                        boost::nowide::cerr << "warning: --import-support-stl provided but no SLA object to attach to." << std::endl;
                }
            }

            // Additive generation: hand the engine the pillars of the support
            // that is already on the plate. They are braced to and counted
            // towards the new pillar's link budget, but never re-emitted.
            const std::string prior_supports_path = opt_path(cli.misc_config, "prior_supports");
            if (printer_technology == ptSLA && !prior_supports_path.empty()) {
                boost::nowide::ifstream pifs(prior_supports_path);
                if (!pifs.good()) {
                    boost::nowide::cerr << "error: failed to open --prior-supports: "
                                        << prior_supports_path << std::endl;
                    return false;
                }
                std::ostringstream ptext;
                ptext << pifs.rdbuf();

                sla::PriorPillars priors;
                std::string prior_err;
                if (!sla::prior_pillars_from_string(ptext.str(), priors, prior_err)) {
                    boost::nowide::cerr << "error: --prior-supports: " << prior_err << std::endl;
                    return false;
                }
                if (!sla_print.attach_prior_pillars(priors))
                    boost::nowide::cerr << "warning: --prior-supports provided but no SLA object to attach to." << std::endl;
                else
                    boost::nowide::cout << "Loaded " << priors.size()
                                        << " prior support pillars from " << prior_supports_path << std::endl;
            }

            if (actions.has("export_preview_pngs") && printer_technology == ptSLA) {
                double scale = actions.opt_float("export_preview_pngs");
                if (scale > 0.)
                    sla_print.set_preview_scale(scale);
            }

            std::string err = print->validate();
            if (!err.empty()) {
                boost::nowide::cerr << err << std::endl;
                return 1;
            }

            std::string outfile = output;

            if (print->empty())
                boost::nowide::cout << "Nothing to print for " << outfile << " . Either the print is empty or no object is fully inside the print volume." << std::endl;
            else
                try {
                std::string outfile_final;

                // Support-only fast path: when the caller asked solely for the
                // support STL (no SLA archive / preview / gcode), stop the SLA
                // pipeline right after the pad step. This skips slice-supports,
                // merge-and-eval and rasterization + sl1 packing, none of which
                // the support/pad mesh depends on.
                const bool support_stl_only = printer_technology == ptSLA
                    && actions.has("export_support_stl")
                    && !actions.has("export_sla")
                    && !actions.has("slice")
                    && !actions.has("export_gcode")
                    && !actions.has("export_preview_pngs");
                // Points-only fast path: stop one step earlier still, right
                // after the support points are computed. Nothing downstream -
                // support tree, pad, slicing, rasterization - is needed to write
                // the point list, and skipping the tree is where the speed comes
                // from. Note this deliberately excludes export_support_stl:
                // asking for both is legal, and the pad stop below already runs
                // past the support point step.
                const bool support_points_only = printer_technology == ptSLA
                    && !export_support_points_path.empty()
                    && !actions.has("export_support_stl")
                    && !actions.has("export_sla")
                    && !actions.has("slice")
                    && !actions.has("export_gcode")
                    && !actions.has("export_preview_pngs");

                if (support_points_only) {
                    PrintBase::TaskParams task_params;
                    task_params.to_object_step = slaposSupportPoints;
                    sla_print.set_task(task_params);
                }
                else if (support_stl_only) {
                    PrintBase::TaskParams task_params;
                    task_params.to_object_step = slaposPad;
                    sla_print.set_task(task_params);
                }
                print->process();
                if (printer_technology == ptFFF) {
                    // The outfile is processed by a PlaceholderParser.
                    const std::string input_file = fff_print.model().objects.empty() ? "" : fff_print.model().objects.front()->input_file;
                    outfile = fff_print.export_gcode(outfile, nullptr, get_thumbnail_generator_cli(input_file));
                    outfile_final = fff_print.print_statistics().finalize_output_path(outfile);
                }
                else {
                    outfile = sla_print.output_filepath(outfile);
                    if (support_stl_only || support_points_only) {
                        // No sl1 archive in either of these modes; keep a
                        // filename stem for the *_support.stl output below.
                        outfile_final = outfile;
                    }
                    else {
                        // We need to finalize the filename beforehand because the export function sets the filename inside the zip metadata
                        outfile_final = sla_print.print_statistics().finalize_output_path(outfile);
                        sla_print.export_print(outfile_final);

                        // Export preview PNGs ZIP if requested. A failure here is
                        // reported and then dropped: the .sl1 above is already
                        // written, and the preview is only ever consumed by the UI,
                        // so failing the whole slice over it would throw away work
                        // the printer file no longer depends on.
                        if (actions.has("export_preview_pngs") && actions.opt_float("export_preview_pngs") > 0.) {
                            auto preview_path = boost::filesystem::path(outfile_final);
                            preview_path.replace_filename(preview_path.stem().string() + "_preview.zip");
                            if (sla_print.export_preview_zip(preview_path.string()))
                                boost::nowide::cout << "Preview ZIP exported to " << preview_path.string() << std::endl;
                            else
                                // Deliberately not the line above: that one is the
                                // agent's archive-done marker, and claiming a file
                                // that is not there would put a false statement into
                                // the job log. The slice still ends successfully.
                                boost::nowide::cerr << "warning: preview ZIP could not be written to "
                                                    << preview_path.string()
                                                    << "; the .sl1 is unaffected and slicing continues." << std::endl;
                        }
                    }

                    // Write the support point list (--export-support-points).
                    if (!export_support_points_path.empty()) {
                        if (sla_print.objects().size() != 1) {
                            boost::nowide::cerr << "error: --export-support-points handles a single "
                                                   "object per file; this print has "
                                                << sla_print.objects().size() << std::endl;
                            return false;
                        }

                        const SLAPrintObject *po = sla_print.objects().front();
                        if (!po->is_step_done(slaposSupportPoints)) {
                            boost::nowide::cerr << "error: --export-support-points: the support point "
                                                   "step did not run" << std::endl;
                            return false;
                        }

                        // Back into the coordinate system of the file the caller
                        // handed in. trafo() is used through its accessor and
                        // inverted whole: it folds in the shrinkage compensation
                        // and a left handed mirroring, and rebuilding it by hand
                        // would quietly lose both.
                        const Transform3d to_object_space = po->trafo().inverse();

                        sla::SupportPoints out_points;
                        const std::vector<sla::SupportPoint> &pts = po->get_support_points();
                        out_points.reserve(pts.size());
                        for (const sla::SupportPoint &sp : pts) {
                            const Vec3d mapped = to_object_space * sp.pos.cast<double>();

                            // trafo() is only invertible while its linear part
                            // is non-singular, and it is built as
                            // Diagonal(correction) * instance.linear() - either
                            // factor can collapse to zero through a zero
                            // shrinkage compensation or a zero instance scale.
                            // Eigen answers a singular inverse with infinities
                            // rather than an exception, so the check has to be
                            // on the result. in_float_range() also covers the
                            // narrowing to float below, which is undefined
                            // behaviour for anything past FLT_MAX.
                            if (!sla::detail::in_float_range(mapped.x()) ||
                                !sla::detail::in_float_range(mapped.y()) ||
                                !sla::detail::in_float_range(mapped.z())) {
                                boost::nowide::cerr
                                    << "error: --export-support-points: the object transform is "
                                       "not invertible (a zero scale or a zero shrinkage "
                                       "compensation), so support point coordinates cannot be "
                                       "mapped back to the input model" << std::endl;
                                return false;
                            }

                            sla::SupportPoint moved = sp;
                            moved.pos = mapped.cast<float>();
                            out_points.push_back(moved);
                        }

                        const sla::SupportTreeConfig scfg = make_support_cfg(po->config());
                        const sla::ModelFingerprint fp = sla::model_fingerprint(*po->model_object());

                        boost::nowide::ofstream ofs(export_support_points_path);
                        if (!ofs.good()) {
                            boost::nowide::cerr << "error: failed to open --export-support-points for "
                                                   "writing: " << export_support_points_path << std::endl;
                            return false;
                        }
                        ofs << sla::support_points_to_string(out_points, fp, scfg) << std::endl;
                        ofs.close();
                        if (!ofs) {
                            boost::nowide::cerr << "error: failed to write --export-support-points: "
                                                << export_support_points_path << std::endl;
                            return false;
                        }

                        boost::nowide::cout << "Support points exported to "
                                            << export_support_points_path << " ("
                                            << out_points.size() << " points)" << std::endl;
                    }

                    // Export support mesh (including pad) as STL if requested
                    if (actions.has("export_support_stl")) {
                        for (const SLAPrintObject* po : sla_print.objects()) {
                            TriangleMesh combined_mesh;
                            bool has_support = false;
                            bool has_pad = false;

                            // Get support mesh if available
                            if (po->is_step_done(slaposSupportTree)) {
                                TriangleMesh support_mesh = po->support_mesh();
                                if (!support_mesh.empty()) {
                                    combined_mesh.merge(support_mesh);
                                    has_support = true;
                                }
                            }

                            // Get pad mesh if available
                            if (po->is_step_done(slaposPad)) {
                                TriangleMesh pad_mesh = po->pad_mesh();
                                if (!pad_mesh.empty()) {
                                    combined_mesh.merge(pad_mesh);
                                    has_pad = true;
                                }
                            }

                            // The pillars this generation grew, for handing to
                            // the next one as --prior-supports. Written even when
                            // the mesh is empty: "nothing grew" is a real answer
                            // the caller has to be able to record.
                            if (!export_support_pillars_path.empty()) {
                                const std::string doc = sla::prior_pillars_to_string(
                                    po->generated_pillars(), po->get_elevation(),
                                    po->prior_attachments());
                                boost::nowide::ofstream pofs(export_support_pillars_path);
                                if (pofs.good()) {
                                    pofs << doc;
                                    boost::nowide::cout << "Support pillars exported to "
                                                        << export_support_pillars_path << " ("
                                                        << po->generated_pillars().size()
                                                        << " pillars)" << std::endl;
                                } else {
                                    boost::nowide::cerr << "Failed to export support pillars to "
                                                        << export_support_pillars_path << std::endl;
                                }
                            }

                            // Braces reaching pillars from earlier generations,
                            // one file each. Separate from the support mesh so
                            // that removing such a pillar can take its brace
                            // with it, leaving the support itself untouched.
                            if (!export_brace_stls_dir.empty()) {
                                boost::system::error_code ec;
                                boost::filesystem::create_directories(export_brace_stls_dir, ec);
                                for (const auto &[prior_id, its] : po->frozen_braces()) {
                                    if (its.empty()) continue;
                                    TriangleMesh bm{its};
                                    boost::filesystem::path bp(export_brace_stls_dir);
                                    bp /= ("brace_" + std::to_string(prior_id) + ".stl");
                                    if (bm.write_binary(bp.string().c_str()))
                                        boost::nowide::cout << "Brace mesh exported to " << bp.string() << std::endl;
                                    else
                                        boost::nowide::cerr << "Failed to export brace mesh to " << bp.string() << std::endl;
                                }
                            }

                            if (!combined_mesh.empty()) {
                                boost::filesystem::path support_path(outfile_final);
                                std::string stem = support_path.stem().string();
                                support_path.replace_filename(stem + "_support.stl");
                                if (combined_mesh.write_binary(support_path.string().c_str())) {
                                    boost::nowide::cout << "Support mesh exported to " << support_path.string();
                                    if (has_support && has_pad)
                                        boost::nowide::cout << " (includes supports and pad)";
                                    else if (has_support)
                                        boost::nowide::cout << " (supports only)";
                                    else if (has_pad)
                                        boost::nowide::cout << " (pad only)";
                                    boost::nowide::cout << std::endl;
                                } else {
                                    boost::nowide::cerr << "Failed to export support mesh to " << support_path.string() << std::endl;
                                }
                            } else {
                                boost::nowide::cout << "No support/pad mesh generated" << std::endl;
                            }
                        }
                    }
                }
                if (!support_stl_only && !support_points_only) {
                    if (outfile != outfile_final) {
                        if (Slic3r::rename_file(outfile, outfile_final)) {
                            boost::nowide::cerr << "Renaming file " << outfile << " to " << outfile_final << " failed" << std::endl;
                            return false;
                        }
                        outfile = outfile_final;
                    }
                    // Run the post-processing scripts if defined.
                    run_post_process_scripts(outfile, fff_print.full_print_config());
                    boost::nowide::cout << "Slicing result exported to " << outfile << std::endl;
                }
            }
            catch (const std::exception& ex) {
                boost::nowide::cerr << ex.what() << std::endl;
                return false;
            }

        }
    }

    return true;
}

}
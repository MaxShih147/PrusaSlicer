///|/ Copyright (c) Prusa Research 2022 - 2023 Tomáš Mészáros @tamasmeszaros
///|/ Copyright (c) 2023 Mimoja @Mimoja
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "SLAArchiveWriter.hpp"

#include "SLAArchiveFormatRegistry.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/Zipper.hpp"

#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>

// Confined to this translation unit: on MSVC this header includes <windows.h>,
// and SLAArchiveWriter.hpp is reachable from SLAPrint.hpp, so letting it into the
// header would put the Win32 macro surface (notably the GDI ::Polygon function,
// which then collides with Slic3r::Polygon) into a large part of the codebase.
#include <tbb/enumerable_thread_specific.h>

namespace Slic3r {

class ThreadBoundRasters::Impl {
public:
    tbb::enumerable_thread_specific<std::unique_ptr<sla::RasterBase>> rasters;
};

ThreadBoundRasters::ThreadBoundRasters() : m_impl(std::make_unique<Impl>()) {}
ThreadBoundRasters::~ThreadBoundRasters() = default;

sla::RasterBase &ThreadBoundRasters::acquire(const RasterFactory &factory)
{
    std::unique_ptr<sla::RasterBase> &slot = m_impl->rasters.local();
    if (slot)
        slot->reset();   // wipe the previous layer before reuse
    else
        slot = factory();

    return *slot;
}

std::unique_ptr<SLAArchiveWriter>
SLAArchiveWriter::create(const std::string &archtype, const SLAPrinterConfig &cfg)
{
    std::unique_ptr<SLAArchiveWriter> ret;
    auto factory = get_writer_factory(archtype.c_str());

    if (factory)
        ret = factory(cfg);

    return ret;
}

// Returns false when the ZIP could not be written. The failure is deliberately
// NOT propagated: the preview is an auxiliary artifact, and by the time we get
// here the .sl1 is already on disk and the whole slice has been paid for. Letting
// a failed preview throw would discard a completed slice over a file the printer
// never reads, and the caller would have to redo minutes of work to get it back.
// The error is logged instead, and the caller decides what to say about it.
bool SLAArchiveWriter::export_preview_zip(const std::string &fname,
                                          const std::string &projectname)
{
    if (m_preview_layers.empty()) return true; // nothing to write is not a failure

    std::string project =
        projectname.empty() ?
            boost::filesystem::path(fname).stem().string() :
            projectname;

    try {
        Zipper zipper{fname, Zipper::FAST_COMPRESSION};

        size_t i = 0;
        for (const sla::EncodedRaster &rst : m_preview_layers) {
            std::string imgname = project + string_printf("%.5d", i++) +
                                  "." + rst.extension();
            zipper.add_entry(imgname.c_str(), rst.data(), rst.size());
        }

        zipper.finalize();
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(error) << "Failed to write preview ZIP: " << e.what();
        return false;
    }

    return true;
}

} // namespace Slic3r

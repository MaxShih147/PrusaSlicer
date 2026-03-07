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

namespace Slic3r {

std::unique_ptr<SLAArchiveWriter>
SLAArchiveWriter::create(const std::string &archtype, const SLAPrinterConfig &cfg)
{
    std::unique_ptr<SLAArchiveWriter> ret;
    auto factory = get_writer_factory(archtype.c_str());

    if (factory)
        ret = factory(cfg);

    return ret;
}

void SLAArchiveWriter::export_preview_zip(const std::string &fname,
                                          const std::string &projectname)
{
    if (m_preview_layers.empty()) return;

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
        throw;
    }
}

} // namespace Slic3r

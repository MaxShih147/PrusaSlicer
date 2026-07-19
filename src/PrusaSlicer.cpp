#ifdef WIN32
    // Why?
    #define _WIN32_WINNT 0x0502
    // The standard Windows includes.
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif // WIN32_LEAN_AND_MEAN
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif // NOMINMAX
    #include <Windows.h>
    #include <wchar.h>
    #ifdef SLIC3R_GUI
    extern "C"
    {
        // Let the NVIDIA and AMD know we want to use their graphics card
        // on a dual graphics card system.
        __declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
        __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
    }
    #endif /* SLIC3R_GUI */
#endif /* WIN32 */

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <cstring>
#include <iostream>
#include <vector>

#include <boost/nowide/args.hpp>

#include "libslic3r/libslic3r.h"
#include "CLI/CLI.hpp"
#ifdef BUNDLE_QA_CRASH_HARNESS
#include "bundle_qa_crash_probe.hpp"
#endif

#include "PrusaSlicer.hpp"

namespace {

// REQ-DEID-006 / tasks 5.1b: libc++abi's default terminate handler prints
// demangled type names (e.g. Slic3r::…). Replace with a brand-neutral abort.
[[noreturn]] void slicer_engine_neutral_terminate() noexcept
{
    std::fputs("slicer-engine: fatal error\n", stderr);
    std::fflush(stderr);
    std::abort();
}

void slicer_engine_install_exception_guards()
{
    std::set_terminate(slicer_engine_neutral_terminate);
}

} // namespace

// __has_feature() is used later for Clang, this is for compatibility with other compilers (such as GCC and MSVC)
#ifndef __has_feature
#   define __has_feature(x) 0
#endif

#if __has_feature(address_sanitizer) || defined(__SANITIZE_ADDRESS__)
extern "C" {
    // Based on https://github.com/google/skia/blob/main/tools/LsanSuppressions.cpp
    const char *__lsan_default_suppressions() {
        return "leak:libfontconfig\n"           // FontConfig looks like it leaks, but it doesn't.
               "leak:libfreetype\n"             // Unsure, appeared upgrading Debian 9->10.
               "leak:libGLX_nvidia.so\n"        // For NVidia driver.
               "leak:libnvidia-glcore.so\n"     // For NVidia driver.
               "leak:libnvidia-tls.so\n"        // For NVidia driver.
               "leak:terminator_CreateDevice\n" // For Intel Vulkan drivers.
               "leak:swrast_dri.so\n"           // For Mesa 3D software driver.
               "leak:amdgpu_dri.so\n"           // For AMD driver.
               "leak:libdrm_amdgpu.so\n"        // For AMD driver.
               "leak:libdbus-1.so\n"            // For D-Bus library. Unsure if it is a leak or not.
            ;
    }
}
#endif

#if defined(SLIC3R_UBSAN)
extern "C" {
    // Enable printing stacktrace by default. It can be disabled by running PrusaSlicer with "UBSAN_OPTIONS=print_stacktrace=0".
    const char *__ubsan_default_options() {
        return "print_stacktrace=1";
    }
}
#endif

#if defined(_MSC_VER) || defined(__MINGW32__)
// Export via slicer_core.def only (no __declspec) so cereal residuals cannot
// share the export surface with an extra dllexport on this entry (W-EXP-1).
extern "C" {
    int __stdcall slicer_run_cli(int argc, wchar_t **argv)
    {
        slicer_engine_install_exception_guards();
        try {
#ifdef BUNDLE_QA_CRASH_HARNESS
            Slic3r::BundleQa::maybe_force_crash();
#endif
            // Convert wchar_t arguments to UTF8.
            std::vector<std::string> 	argv_narrow;
            std::vector<char*>			argv_ptrs(argc + 1, nullptr);
            for (size_t i = 0; i < argc; ++ i)
                argv_narrow.emplace_back(boost::nowide::narrow(argv[i]));
            for (size_t i = 0; i < argc; ++ i)
                argv_ptrs[i] = argv_narrow[i].data();
            // Call the UTF8 main.
            return Slic3r::CLI::run(argc, argv_ptrs.data());
        } catch (...) {
            slicer_engine_neutral_terminate();
        }
    }
}
#else /* _MSC_VER */
int main(int argc, char **argv)
{
    slicer_engine_install_exception_guards();
    try {
#ifdef BUNDLE_QA_CRASH_HARNESS
        Slic3r::BundleQa::maybe_force_crash();
#endif
        return Slic3r::CLI::run(argc, argv);
    } catch (...) {
        slicer_engine_neutral_terminate();
    }
}
#endif /* _MSC_VER */

// Compile-time QA crash probe — only built when BUNDLE_QA_CRASH_HARNESS is ON.
// See openspec/.../crash-harness-forensics.md and windows-policy.md.

#ifdef BUNDLE_QA_CRASH_HARNESS

#include "bundle_qa_crash_probe.hpp"

#include <cstdlib>
#include <cstring>
#include <stdexcept>

#if defined(_MSC_VER)
#pragma optimize("", off)
#endif

namespace Slic3r {
namespace BundleQa {

#if defined(__GNUC__) || defined(__clang__)
__attribute__((noinline, optnone))
#elif defined(_MSC_VER)
__declspec(noinline)
#endif
static void force_stack_overflow()
{
    volatile char pad[8192];
    pad[0] = 1;
    force_stack_overflow();
}

#if defined(__GNUC__) || defined(__clang__)
__attribute__((noinline, optnone))
#elif defined(_MSC_VER)
__declspec(noinline)
#endif
static void force_segfault()
{
    volatile int *p = nullptr;
    *p = 1;
}

struct ForcedException : public std::runtime_error
{
    ForcedException() : std::runtime_error("bundle qa forced exception") {}
};

void maybe_force_crash()
{
    const char *mode = std::getenv("BUNDLE_QA_CRASH_MODE");
    const char *legacy = std::getenv("BUNDLE_FORCE_PRUSA_STACK_OVERFLOW");
    if ((!mode || !mode[0]) && legacy && legacy[0] == '1')
        mode = "overflow";
    if (!mode || !mode[0])
        return;

    if (std::strcmp(mode, "overflow") == 0) {
        force_stack_overflow();
    } else if (std::strcmp(mode, "segfault") == 0) {
        force_segfault();
    } else if (std::strcmp(mode, "exception") == 0) {
        // noexcept + throw → std::terminate → abort (WER / ReportCrash)
        auto boom = []() noexcept {
            throw ForcedException();
        };
        boom();
    }
}

} // namespace BundleQa
} // namespace Slic3r

#if defined(_MSC_VER)
#pragma optimize("", on)
#endif

#endif // BUNDLE_QA_CRASH_HARNESS

#pragma once

// Compile-time QA crash harness (REQ-DEID-009 / D7).
// Only compiled when BUNDLE_QA_CRASH_HARNESS is defined.
// Mode selection (when harness is present): BUNDLE_QA_CRASH_MODE=overflow|segfault|exception
// Legacy: BUNDLE_FORCE_PRUSA_STACK_OVERFLOW=1 → overflow

#ifdef BUNDLE_QA_CRASH_HARNESS
namespace Slic3r {
namespace BundleQa {
// Call once early from CLI entry. No-op body is not linked in consumer builds.
void maybe_force_crash();
} // namespace BundleQa
} // namespace Slic3r
#endif

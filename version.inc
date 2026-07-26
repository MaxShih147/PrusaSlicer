# Included by CMakeLists, edited by the build script
# De-id PoC (2026-07-17): neutral display names per naming-manifest / windows-policy.

set(SLIC3R_APP_NAME "Slicer Engine")
set(SLIC3R_APP_KEY "SlicerEngine")
set(SLIC3R_VERSION "1.0.5")
set(SLIC3R_BUILD_ID "Slicer Engine ${SLIC3R_VERSION}")
set(SLIC3R_RC_VERSION "1,0,5,0")
set(SLIC3R_RC_VERSION_DOTS "1.0.5.0")
# Windows VERSIONINFO (naming-manifest §2.2 / windows-policy W-VER-1)
set(SLIC3R_COMPANY_NAME "Phrozen")
set(SLIC3R_INTERNAL_NAME "slicer-engine")
set(SLIC3R_ORIGINAL_FILENAME "slicer-engine.exe")
set(SLIC3R_COPYRIGHT "Copyright (c) Phrozen Tech Co., Ltd.")
# SLIC3R_APP_ICON is set in root CMakeLists.txt after SLIC3R_RESOURCES_DIR
# (resources/icons/slicer-engine.ico — PE icon SoT for PrusaSlicer.rc.in).
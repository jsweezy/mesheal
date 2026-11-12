# vcpkg triplet for Mesheal's Windows release build: static libraries, dynamic C runtime (/MD),
# release builds only. Debug builds are skipped: Mesheal does not use them, they double the build
# time, and MPFR's debug configure hangs on the GitHub Windows runners.
set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)
set(VCPKG_BUILD_TYPE release)

# Static, release-only libraries for the Blender module on Linux. vcpkg compiles Linux libraries
# with -fPIC, so they can be linked into the module (a shared library).
set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)
set(VCPKG_BUILD_TYPE release)
set(VCPKG_CMAKE_SYSTEM_NAME Linux)

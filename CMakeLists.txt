cmake_minimum_required(VERSION 3.20)

# RELEASE VERSION: edit the VERSION integer below and MESHEAL_VERSION_DATE together
# (see RELEASE_PROCESS.md). Versions are simple integers: 1, 2, 3, ...
project(
    Mesheal
    VERSION 4
    DESCRIPTION "Mesheal — STL Repair Tool for Simulation Meshes"
    LANGUAGES CXX C
)

set(MESHEAL_VERSION_DATE "2026-11-13")  # ISO release date (YYYY-MM-DD)

# ------------------------------------------------------------------------------
# Language Standard & Global Settings
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ------------------------------------------------------------------------------
# Build Options
# ------------------------------------------------------------------------------
option(MESHEAL_BUILD_TESTS "Build Mesheal Catch2 unit tests" ON)
option(MESHEAL_BUILD_GUI   "Build Mesheal Polyscope/ImGui desktop application" ON)
option(MESHEAL_BUILD_PYTHON "Build the _mesheal Python module (Blender extension)" OFF)
option(MESHEAL_USE_TBB     "Use oneTBB through CGAL when found" ON)

# A Python extension module is a shared library: every static library linked into it needs PIC.
if(MESHEAL_BUILD_PYTHON)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

# ------------------------------------------------------------------------------
# Project-Wide Compiler Warnings (applied only to Mesheal targets)
# ------------------------------------------------------------------------------
add_library(mesheal_warnings INTERFACE)
add_library(mesheal::warnings ALIAS mesheal_warnings)

if(MSVC)
    target_compile_options(mesheal_warnings INTERFACE /W4 /permissive-)
    # All targets, dependencies included: CGAL objects exceed MSVC's default section limit,
    # sources are UTF-8 (e.g. "mm³"), and windows.h must not define min/max macros.
    add_compile_options(/bigobj /utf-8)
    add_compile_definitions(NOMINMAX WIN32_LEAN_AND_MEAN _USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS)
    if(MESHEAL_BUILD_PYTHON)
        # The module runs on the C++ runtime Blender ships, which can be older than this compiler's;
        # std::mutex built by a newer compiler crashes on an older runtime unless this is defined.
        add_compile_definitions(_DISABLE_CONSTEXPR_MUTEX_CONSTRUCTOR)
    endif()
else()
    target_compile_options(mesheal_warnings INTERFACE
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wsign-conversion
    )
endif()

# ------------------------------------------------------------------------------
# Testing Setup (CTest)
# ------------------------------------------------------------------------------
if(MESHEAL_BUILD_TESTS)
    include(CTest)
    enable_testing()
endif()

# ------------------------------------------------------------------------------
# Third-Party Dependencies (FetchContent)
# ------------------------------------------------------------------------------
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(Dependencies)
include(Version)

# ------------------------------------------------------------------------------
# Mesheal Source Modules (each with co-located Test/ directory)
# ------------------------------------------------------------------------------
add_subdirectory(src/core)
add_subdirectory(src/mesh)
add_subdirectory(src/io)
add_subdirectory(src/diagnostics)
add_subdirectory(src/repair)
add_subdirectory(src/app)

if(MESHEAL_BUILD_GUI)
    add_subdirectory(src/mockup)
endif()

if(MESHEAL_BUILD_PYTHON)
    add_subdirectory(src/python)
endif()

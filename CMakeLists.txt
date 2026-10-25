cmake_minimum_required(VERSION 3.20)

project(
    Ogun
    VERSION 0.1.0
    DESCRIPTION "Ogun — STL Repair Tool for Simulation Meshes"
    LANGUAGES CXX C
)

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
option(OGUN_BUILD_TESTS "Build Ogun Catch2 unit tests" ON)
option(OGUN_BUILD_GUI   "Build Ogun Polyscope/ImGui desktop application" ON)

# ------------------------------------------------------------------------------
# Project-Wide Compiler Warnings (applied only to Ogun targets)
# ------------------------------------------------------------------------------
add_library(ogun_warnings INTERFACE)
add_library(ogun::warnings ALIAS ogun_warnings)

if(MSVC)
    target_compile_options(ogun_warnings INTERFACE /W4 /permissive-)
else()
    target_compile_options(ogun_warnings INTERFACE
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
if(OGUN_BUILD_TESTS)
    include(CTest)
    enable_testing()
endif()

# ------------------------------------------------------------------------------
# Third-Party Dependencies (FetchContent)
# ------------------------------------------------------------------------------
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(Dependencies)

# ------------------------------------------------------------------------------
# Ogun Source Modules (each with co-located Test/ directory)
# ------------------------------------------------------------------------------
add_subdirectory(src/core)
add_subdirectory(src/mesh)
add_subdirectory(src/io)
add_subdirectory(src/diagnostics)
add_subdirectory(src/repair)
add_subdirectory(src/app)

if(OGUN_BUILD_GUI)
    add_subdirectory(src/mockup)
endif()

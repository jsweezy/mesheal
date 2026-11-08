# Generates mesheal/Version.hpp (release version + build info) into the build tree.
# The version comes from project(VERSION ...) and MESHEAL_VERSION_DATE in the
# top-level CMakeLists.txt. Everything is captured at configure time, including
# the build timestamp, so re-run cmake to refresh it.

set(MESHEAL_VERSION_NUMBER "${PROJECT_VERSION_MAJOR}")

string(TIMESTAMP MESHEAL_BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S UTC" UTC)

if(CMAKE_BUILD_TYPE)
    set(MESHEAL_BUILD_TYPE "${CMAKE_BUILD_TYPE}")
    string(TOUPPER "${CMAKE_BUILD_TYPE}" _mesheal_cfg)
else()
    set(MESHEAL_BUILD_TYPE "unspecified")
    set(_mesheal_cfg "")
endif()

string(STRIP "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${_mesheal_cfg}}" MESHEAL_CXX_FLAGS)
string(STRIP "${CMAKE_EXE_LINKER_FLAGS} ${CMAKE_EXE_LINKER_FLAGS_${_mesheal_cfg}}" MESHEAL_LINKER_FLAGS)
set(MESHEAL_CXX_STANDARD "${CMAKE_CXX_STANDARD}")
set(MESHEAL_COMPILER_ID "${CMAKE_CXX_COMPILER_ID}")
set(MESHEAL_COMPILER_VERSION "${CMAKE_CXX_COMPILER_VERSION}")
set(MESHEAL_PLATFORM "${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_PROCESSOR}")

# Dependencies become brace-initialised C++ entries: {"name", "version"},
set(MESHEAL_DEPENDENCY_ENTRIES "")
foreach(_dep IN LISTS MESHEAL_DEPENDENCY_LIST)
    string(REPLACE "|" ")mesheal\", R\"mesheal(" _dep_cpp "${_dep}")
    string(APPEND MESHEAL_DEPENDENCY_ENTRIES "    {R\"mesheal(${_dep_cpp})mesheal\"},\n")
endforeach()

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Version.hpp.in"
    "${CMAKE_BINARY_DIR}/generated/mesheal/Version.hpp"
    @ONLY
)

add_library(mesheal_version INTERFACE)
add_library(mesheal::version ALIAS mesheal_version)
target_include_directories(mesheal_version INTERFACE "${CMAKE_BINARY_DIR}/generated")

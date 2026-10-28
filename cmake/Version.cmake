# Generates ogun/Version.hpp (release version + build info) into the build tree.
# The version comes from project(VERSION ...) and OGUN_VERSION_DATE in the
# top-level CMakeLists.txt. Everything is captured at configure time, including
# the build timestamp, so re-run cmake to refresh it.

set(OGUN_VERSION_NUMBER "${PROJECT_VERSION_MAJOR}")

string(TIMESTAMP OGUN_BUILD_TIMESTAMP "%Y-%m-%d %H:%M:%S UTC" UTC)

if(CMAKE_BUILD_TYPE)
    set(OGUN_BUILD_TYPE "${CMAKE_BUILD_TYPE}")
    string(TOUPPER "${CMAKE_BUILD_TYPE}" _ogun_cfg)
else()
    set(OGUN_BUILD_TYPE "unspecified")
    set(_ogun_cfg "")
endif()

string(STRIP "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${_ogun_cfg}}" OGUN_CXX_FLAGS)
string(STRIP "${CMAKE_EXE_LINKER_FLAGS} ${CMAKE_EXE_LINKER_FLAGS_${_ogun_cfg}}" OGUN_LINKER_FLAGS)
set(OGUN_CXX_STANDARD "${CMAKE_CXX_STANDARD}")
set(OGUN_COMPILER_ID "${CMAKE_CXX_COMPILER_ID}")
set(OGUN_COMPILER_VERSION "${CMAKE_CXX_COMPILER_VERSION}")
set(OGUN_PLATFORM "${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_PROCESSOR}")

# Dependencies become brace-initialised C++ entries: {"name", "version"},
set(OGUN_DEPENDENCY_ENTRIES "")
foreach(_dep IN LISTS OGUN_DEPENDENCY_LIST)
    string(REPLACE "|" ")ogun\", R\"ogun(" _dep_cpp "${_dep}")
    string(APPEND OGUN_DEPENDENCY_ENTRIES "    {R\"ogun(${_dep_cpp})ogun\"},\n")
endforeach()

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Version.hpp.in"
    "${CMAKE_BINARY_DIR}/generated/ogun/Version.hpp"
    @ONLY
)

add_library(ogun_version INTERFACE)
add_library(ogun::version ALIAS ogun_version)
target_include_directories(ogun_version INTERFACE "${CMAKE_BINARY_DIR}/generated")

include(FetchContent)

# Ensure compatibility with third-party FetchContent subprojects under CMake 4.x+
if(NOT DEFINED CMAKE_POLICY_VERSION_MINIMUM)
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "Minimum policy version for subprojects" FORCE)
endif()
if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)
endif()

# Once dependencies are downloaded into _deps/, do not re-fetch on reconfigure
set(FETCHCONTENT_UPDATES_DISCONNECTED ON CACHE BOOL "Skip network updates once populated" FORCE)

# Build third-party libraries statically by default
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libraries" FORCE)

# Dependency name/version pairs shown in the About dialog (see Version.cmake)
set(MESHEAL_DEPENDENCY_LIST "")
function(mesheal_record_dependency name version)
    list(APPEND MESHEAL_DEPENDENCY_LIST "${name}|${version}")
    set(MESHEAL_DEPENDENCY_LIST "${MESHEAL_DEPENDENCY_LIST}" PARENT_SCOPE)
endfunction()

# Helper to mark a third-party target's include directories as SYSTEM (-isystem)
# so strict compiler warnings on Mesheal targets do not trigger inside dependencies.
function(mesheal_mark_target_includes_system target_name)
    if(TARGET "${target_name}")
        get_target_property(_aliased "${target_name}" ALIASED_TARGET)
        if(_aliased)
            set(target_name "${_aliased}")
        endif()
        get_target_property(_inc_dirs "${target_name}" INTERFACE_INCLUDE_DIRECTORIES)
        if(_inc_dirs)
            set_target_properties("${target_name}" PROPERTIES
                INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_inc_dirs}"
            )
        endif()
    endif()
endfunction()

# ------------------------------------------------------------------------------
# 1. Catch2 (v3) — Unit Testing Framework
# ------------------------------------------------------------------------------
if(MESHEAL_BUILD_TESTS)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.7.1
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(Catch2)
    list(APPEND CMAKE_MODULE_PATH "${catch2_SOURCE_DIR}/extras")
    include(Catch)
    mesheal_mark_target_includes_system(Catch2::Catch2)
    mesheal_mark_target_includes_system(Catch2::Catch2WithMain)
    mesheal_record_dependency("Catch2 (tests)" "3.7.1")
endif()

# ------------------------------------------------------------------------------
# 2. libigl (v2.5.0) + Eigen3 — Matrix Geometry & STL I/O
# ------------------------------------------------------------------------------
set(LIBIGL_BUILD_TESTS     OFF CACHE BOOL "" FORCE)
set(LIBIGL_BUILD_TUTORIALS OFF CACHE BOOL "" FORCE)
set(LIBIGL_INSTALL         OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG        v2.5.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(libigl)
mesheal_mark_target_includes_system(Eigen3::Eigen)
mesheal_mark_target_includes_system(igl::core)
mesheal_record_dependency("libigl" "2.5.0")
set(_mesheal_eigen_version "3.4.0")
set(_mesheal_eigen_macros "${eigen_SOURCE_DIR}/Eigen/src/Core/util/Macros.h")
if(EXISTS "${_mesheal_eigen_macros}")
    file(STRINGS "${_mesheal_eigen_macros}" _mesheal_eigen_lines
         REGEX "^#define EIGEN_(WORLD|MAJOR|MINOR)_VERSION [0-9]+")
    string(REGEX MATCHALL "[0-9]+" _mesheal_eigen_parts "${_mesheal_eigen_lines}")
    list(JOIN _mesheal_eigen_parts "." _mesheal_eigen_version)
endif()
mesheal_record_dependency("Eigen" "${_mesheal_eigen_version}")

# ------------------------------------------------------------------------------
# 3. Manifold (v3.0.1) — Manifold Mesh Kernel & MeshGL Interop
# ------------------------------------------------------------------------------
set(MANIFOLD_TEST          OFF CACHE BOOL "" FORCE)
set(MANIFOLD_CROSS_SECTION OFF CACHE BOOL "" FORCE)
set(MANIFOLD_EXPORT        OFF CACHE BOOL "" FORCE)
set(MANIFOLD_CBIND         OFF CACHE BOOL "" FORCE)
set(MANIFOLD_PYBIND        OFF CACHE BOOL "" FORCE)
set(MANIFOLD_JSBIND        OFF CACHE BOOL "" FORCE)
set(MANIFOLD_PAR           OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    manifold
    GIT_REPOSITORY https://github.com/elalish/manifold.git
    GIT_TAG        v3.0.1
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(manifold)
mesheal_mark_target_includes_system(manifold::manifold)
mesheal_record_dependency("Manifold" "3.0.1")

# ------------------------------------------------------------------------------
# 3b. CGAL — Halfedge Mesh Repair (system install preferred, FetchContent fallback)
# ------------------------------------------------------------------------------
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
# Mesheal needs CGAL 6.2+ (autorefinement with snap rounding, optional-returning property maps).
# Older system installs (e.g. Ubuntu 24.04 ships 5.6) are skipped and CGAL 6.2.1 is downloaded.
find_package(CGAL 6.2 CONFIG QUIET)
if(NOT CGAL_FOUND)
    FetchContent_Declare(
        cgal
        URL https://github.com/CGAL/cgal/releases/download/v6.2.1/CGAL-6.2.1.tar.xz
        # Header-only: just unpack it and use its CGALConfig.cmake; don't add CGAL's own project
        SOURCE_SUBDIR cmake_ignore_subdir
    )
    FetchContent_MakeAvailable(cgal)
    find_package(CGAL CONFIG REQUIRED PATHS "${cgal_SOURCE_DIR}" NO_DEFAULT_PATH)
endif()
mesheal_mark_target_includes_system(CGAL::CGAL)
mesheal_record_dependency("CGAL" "${CGAL_VERSION}")
# The Python module turns TBB off: Blender ships its own libtbb and two copies in one process are risky.
if(MESHEAL_USE_TBB)
    find_package(TBB CONFIG QUIET HINTS /opt/homebrew/opt/tbb /usr/local/opt/tbb)
endif()
if(MESHEAL_USE_TBB AND TBB_FOUND)
    include(CGAL_TBB_support)
    mesheal_mark_target_includes_system(TBB::tbb)
    mesheal_mark_target_includes_system(TBB::tbbmalloc)
    mesheal_record_dependency("oneTBB" "${TBB_VERSION}")
endif()
if(Boost_VERSION)
    mesheal_record_dependency("Boost" "${Boost_VERSION}")
elseif(Boost_VERSION_STRING)
    mesheal_record_dependency("Boost" "${Boost_VERSION_STRING}")
endif()
if(GMP_VERSION)
    mesheal_record_dependency("GMP" "${GMP_VERSION}")
endif()
if(MPFR_VERSION)
    mesheal_record_dependency("MPFR" "${MPFR_VERSION}")
endif()

# ------------------------------------------------------------------------------
# 4. Polyscope + Dear ImGui & portable-file-dialogs (GUI Dependencies)
# ------------------------------------------------------------------------------
if(MESHEAL_BUILD_GUI)
    FetchContent_Declare(
        polyscope
        GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
        GIT_TAG        v2.4.0
        GIT_SHALLOW    TRUE
        GIT_SUBMODULES ""
        PATCH_COMMAND  git submodule update --init --recursive --depth 1
    )
    FetchContent_MakeAvailable(polyscope)
    mesheal_mark_target_includes_system(polyscope)
    mesheal_mark_target_includes_system(imgui)
    mesheal_mark_target_includes_system(glm)
    mesheal_record_dependency("Polyscope" "2.4.0")
    set(_mesheal_imgui_version "unknown")
    file(STRINGS "${polyscope_SOURCE_DIR}/deps/imgui/imgui/imgui.h" _mesheal_imgui_line
         REGEX "^#define IMGUI_VERSION +\"")
    if(_mesheal_imgui_line MATCHES "\"([^\"]+)\"")
        set(_mesheal_imgui_version "${CMAKE_MATCH_1}")
    endif()
    mesheal_record_dependency("Dear ImGui" "${_mesheal_imgui_version}")

    FetchContent_Declare(
        portable_file_dialogs
        GIT_REPOSITORY      https://github.com/samhocevar/portable-file-dialogs.git
        GIT_TAG             main
        GIT_SHALLOW         TRUE
        UPDATE_DISCONNECTED TRUE
        SOURCE_SUBDIR       cmake_ignore_subdir
    )
    FetchContent_MakeAvailable(portable_file_dialogs)

    if(NOT TARGET portable_file_dialogs)
        add_library(portable_file_dialogs INTERFACE)
        target_include_directories(portable_file_dialogs SYSTEM INTERFACE
            "${portable_file_dialogs_SOURCE_DIR}"
        )
    endif()
    mesheal_record_dependency("portable-file-dialogs" "main (unpinned)")
endif()

# ------------------------------------------------------------------------------
# 5. nanobind — Python bindings (only for the _mesheal module)
# ------------------------------------------------------------------------------
if(MESHEAL_BUILD_PYTHON)
    find_package(Python 3.10 COMPONENTS Interpreter Development.Module REQUIRED)
    FetchContent_Declare(
        nanobind
        GIT_REPOSITORY https://github.com/wjakob/nanobind.git
        GIT_TAG        v2.4.0
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(nanobind)
    mesheal_record_dependency("nanobind" "2.4.0")
endif()

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
set(OGUN_DEPENDENCY_LIST "")
function(ogun_record_dependency name version)
    list(APPEND OGUN_DEPENDENCY_LIST "${name}|${version}")
    set(OGUN_DEPENDENCY_LIST "${OGUN_DEPENDENCY_LIST}" PARENT_SCOPE)
endfunction()

# Helper to mark a third-party target's include directories as SYSTEM (-isystem)
# so strict compiler warnings on Ogun targets do not trigger inside dependencies.
function(ogun_mark_target_includes_system target_name)
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
if(OGUN_BUILD_TESTS)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.7.1
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(Catch2)
    list(APPEND CMAKE_MODULE_PATH "${catch2_SOURCE_DIR}/extras")
    include(Catch)
    ogun_mark_target_includes_system(Catch2::Catch2)
    ogun_mark_target_includes_system(Catch2::Catch2WithMain)
    ogun_record_dependency("Catch2 (tests)" "3.7.1")
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
ogun_mark_target_includes_system(Eigen3::Eigen)
ogun_mark_target_includes_system(igl::core)
ogun_record_dependency("libigl" "2.5.0")
set(_ogun_eigen_version "3.4.0")
set(_ogun_eigen_macros "${eigen_SOURCE_DIR}/Eigen/src/Core/util/Macros.h")
if(EXISTS "${_ogun_eigen_macros}")
    file(STRINGS "${_ogun_eigen_macros}" _ogun_eigen_lines
         REGEX "^#define EIGEN_(WORLD|MAJOR|MINOR)_VERSION [0-9]+")
    string(REGEX MATCHALL "[0-9]+" _ogun_eigen_parts "${_ogun_eigen_lines}")
    list(JOIN _ogun_eigen_parts "." _ogun_eigen_version)
endif()
ogun_record_dependency("Eigen" "${_ogun_eigen_version}")

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
ogun_mark_target_includes_system(manifold::manifold)
ogun_record_dependency("Manifold" "3.0.1")

# ------------------------------------------------------------------------------
# 3b. CGAL — Halfedge Mesh Repair (system install preferred, FetchContent fallback)
# ------------------------------------------------------------------------------
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
find_package(CGAL CONFIG QUIET)
if(NOT CGAL_FOUND)
    FetchContent_Declare(
        cgal
        URL https://github.com/CGAL/cgal/releases/download/v6.0.1/CGAL-6.0.1.tar.xz
    )
    FetchContent_MakeAvailable(cgal)
    find_package(CGAL CONFIG REQUIRED PATHS "${cgal_SOURCE_DIR}" NO_DEFAULT_PATH)
endif()
ogun_mark_target_includes_system(CGAL::CGAL)
ogun_record_dependency("CGAL" "${CGAL_VERSION}")
find_package(TBB CONFIG QUIET HINTS /opt/homebrew/opt/tbb /usr/local/opt/tbb)
if(TBB_FOUND)
    include(CGAL_TBB_support)
    ogun_mark_target_includes_system(TBB::tbb)
    ogun_mark_target_includes_system(TBB::tbbmalloc)
    ogun_record_dependency("oneTBB" "${TBB_VERSION}")
endif()
if(Boost_VERSION)
    ogun_record_dependency("Boost" "${Boost_VERSION}")
elseif(Boost_VERSION_STRING)
    ogun_record_dependency("Boost" "${Boost_VERSION_STRING}")
endif()
if(GMP_VERSION)
    ogun_record_dependency("GMP" "${GMP_VERSION}")
endif()
if(MPFR_VERSION)
    ogun_record_dependency("MPFR" "${MPFR_VERSION}")
endif()

# ------------------------------------------------------------------------------
# 4. Polyscope + Dear ImGui & portable-file-dialogs (GUI Dependencies)
# ------------------------------------------------------------------------------
if(OGUN_BUILD_GUI)
    FetchContent_Declare(
        polyscope
        GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
        GIT_TAG        v2.4.0
        GIT_SHALLOW    TRUE
        GIT_SUBMODULES ""
        PATCH_COMMAND  git submodule update --init --recursive --depth 1
    )
    FetchContent_MakeAvailable(polyscope)
    ogun_mark_target_includes_system(polyscope)
    ogun_mark_target_includes_system(imgui)
    ogun_mark_target_includes_system(glm)
    ogun_record_dependency("Polyscope" "2.4.0")
    set(_ogun_imgui_version "unknown")
    file(STRINGS "${polyscope_SOURCE_DIR}/deps/imgui/imgui/imgui.h" _ogun_imgui_line
         REGEX "^#define IMGUI_VERSION +\"")
    if(_ogun_imgui_line MATCHES "\"([^\"]+)\"")
        set(_ogun_imgui_version "${CMAKE_MATCH_1}")
    endif()
    ogun_record_dependency("Dear ImGui" "${_ogun_imgui_version}")

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
    ogun_record_dependency("portable-file-dialogs" "main (unpinned)")
endif()

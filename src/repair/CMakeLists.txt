add_library(ogun_repair STATIC
    GlobalRepair.cpp
    HoleFiller.cpp
    IntersectionHealer.cpp
    MeshCleanup.cpp
    NonManifoldSplit.cpp
    RepairMetrics.cpp
    ShellMerge.cpp
    SliverHealer.cpp
    SurfaceRebuild.cpp
)
add_library(ogun::repair ALIAS ogun_repair)

target_include_directories(ogun_repair
    PUBLIC
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(ogun_repair
    PUBLIC
        ogun::mesh
        ogun::diagnostics
        CGAL::CGAL
    PRIVATE
        ogun::warnings
)

if(OGUN_BUILD_TESTS)
    add_subdirectory(Test)
endif()

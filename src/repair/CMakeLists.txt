add_library(mesheal_repair STATIC
    FixAll.cpp
    FixAllMerge.cpp
    FixAllReport.cpp
    FixAllStages.cpp
    GlobalRepair.cpp
    HoleFiller.cpp
    IntersectionHealer.cpp
    MergePlan.cpp
    MeshCleanup.cpp
    NonManifoldSplit.cpp
    Orientation.cpp
    RepairMetrics.cpp
    ShellMerge.cpp
    SliverHealer.cpp
    SurfaceRebuild.cpp
    TopologyCleanup.cpp
)
add_library(mesheal::repair ALIAS mesheal_repair)

target_include_directories(mesheal_repair
    PUBLIC
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(mesheal_repair
    PUBLIC
        mesheal::mesh
        mesheal::diagnostics
        CGAL::CGAL
        $<$<TARGET_EXISTS:CGAL::TBB_support>:CGAL::TBB_support>
    PRIVATE
        mesheal::warnings
)

if(MESHEAL_BUILD_TESTS)
    add_subdirectory(Test)
endif()

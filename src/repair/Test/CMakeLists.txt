add_executable(mesheal_repair_tests
    FixAllMergeTests.cpp
    FixAllScopeTests.cpp
    FixAllTests.cpp
    HoleFillingTests.cpp
    IntersectionHealerTests.cpp
    MeshCleanupTests.cpp
    NonManifoldSplitTests.cpp
    OrientationTests.cpp
    RepairMetricsTests.cpp
    ShellMergeTests.cpp
    SliverHealerTests.cpp
    SurfaceRebuildTests.cpp
    TopologyCleanupTests.cpp
)

target_link_libraries(mesheal_repair_tests
    PRIVATE
        mesheal::repair
        mesheal::io
        Catch2::Catch2WithMain
        mesheal::warnings
)

catch_discover_tests(mesheal_repair_tests)

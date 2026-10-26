add_executable(ogun_repair_tests
    HoleFillingTests.cpp
    IntersectionHealerTests.cpp
    NonManifoldSplitTests.cpp
    ShellMergeTests.cpp
    SliverHealerTests.cpp
)

target_link_libraries(ogun_repair_tests
    PRIVATE
        ogun::repair
        ogun::io
        Catch2::Catch2WithMain
        ogun::warnings
)

catch_discover_tests(ogun_repair_tests)

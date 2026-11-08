add_executable(mesheal_diagnostics_tests
    ThinFaceCheckerTests.cpp
    MeshChecksTests.cpp
    TriTriIntersectTests.cpp
    SelfIntersectionTests.cpp
    HistogramTests.cpp
    HoleLoopsTests.cpp
    ShellNestingTests.cpp
    ShellRelationsTests.cpp
    ShellRolesTests.cpp
)

target_link_libraries(mesheal_diagnostics_tests
    PRIVATE
        mesheal::io
        mesheal::diagnostics
        Catch2::Catch2WithMain
        mesheal::warnings
)

catch_discover_tests(mesheal_diagnostics_tests)

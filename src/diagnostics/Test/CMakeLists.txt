add_executable(ogun_diagnostics_tests
    ThinFaceCheckerTests.cpp
    MeshChecksTests.cpp
    TriTriIntersectTests.cpp
    SelfIntersectionTests.cpp
    HistogramTests.cpp
    HoleLoopsTests.cpp
    ShellNestingTests.cpp
)

target_link_libraries(ogun_diagnostics_tests
    PRIVATE
        ogun::io
        ogun::diagnostics
        Catch2::Catch2WithMain
        ogun::warnings
)

catch_discover_tests(ogun_diagnostics_tests)

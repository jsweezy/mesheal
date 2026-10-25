add_executable(ogun_diagnostics_tests
    ThinFaceCheckerTests.cpp
    MeshChecksTests.cpp
    HistogramTests.cpp
)

target_link_libraries(ogun_diagnostics_tests
    PRIVATE
        ogun::diagnostics
        ogun::io
        Catch2::Catch2WithMain
        ogun::warnings
)

catch_discover_tests(ogun_diagnostics_tests)

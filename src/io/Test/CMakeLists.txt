add_executable(ogun_io_tests
    ConfigDirectoryTests.cpp
    StlReaderTests.cpp
)

target_link_libraries(ogun_io_tests
    PRIVATE
        ogun::io
        Catch2::Catch2WithMain
        ogun::warnings
)

catch_discover_tests(ogun_io_tests)

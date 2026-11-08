add_executable(mesheal_io_tests
    ConfigDirectoryTests.cpp
    StlReaderTests.cpp
    StlWriterTests.cpp
)

target_link_libraries(mesheal_io_tests
    PRIVATE
        mesheal::io
        Catch2::Catch2WithMain
        mesheal::warnings
)

catch_discover_tests(mesheal_io_tests)

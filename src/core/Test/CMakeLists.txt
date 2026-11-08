add_executable(mesheal_core_tests
    CoreTests.cpp
)

target_link_libraries(mesheal_core_tests
    PRIVATE
        mesheal::core
        Catch2::Catch2WithMain
        mesheal::warnings
)

catch_discover_tests(mesheal_core_tests)

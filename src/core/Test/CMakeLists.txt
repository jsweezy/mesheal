add_executable(ogun_core_tests
    CoreTests.cpp
)

target_link_libraries(ogun_core_tests
    PRIVATE
        ogun::core
        Catch2::Catch2WithMain
        ogun::warnings
)

catch_discover_tests(ogun_core_tests)

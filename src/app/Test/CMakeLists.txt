add_executable(ogun_app_tests
    AppControllerTests.cpp
)

target_link_libraries(ogun_app_tests
    PRIVATE
        ogun::app_controller
        Catch2::Catch2WithMain
        ogun::warnings
)

catch_discover_tests(ogun_app_tests)

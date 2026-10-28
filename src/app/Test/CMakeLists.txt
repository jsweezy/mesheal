add_executable(ogun_app_tests
    AppControllerTests.cpp
    DefectBrowserTests.cpp
    FaceSelectionTests.cpp
    HealthModelTests.cpp
    HoleRepairTests.cpp
    IntersectionRepairTests.cpp
    ShellBrowserTests.cpp
    UserPreferencesTests.cpp
)

target_link_libraries(ogun_app_tests
    PRIVATE
        ogun::app_controller
        Catch2::Catch2WithMain
        ogun::warnings
)

catch_discover_tests(ogun_app_tests)

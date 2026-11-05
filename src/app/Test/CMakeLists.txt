add_executable(ogun_app_tests
    AboutInfoTests.cpp
    AppControllerTests.cpp
    CliRunnerTests.cpp
    DefectBrowserTests.cpp
    FaceSelectionTests.cpp
    FileOpenQueueTests.cpp
    FixAllPanelModelTests.cpp
    FixAllRepairTests.cpp
    FramePacerTests.cpp
    GuiLayoutTests.cpp
    HealthModelTests.cpp
    HoleRepairTests.cpp
    IntersectionRepairTests.cpp
    MouseMappingTests.cpp
    RecentFilesTests.cpp
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

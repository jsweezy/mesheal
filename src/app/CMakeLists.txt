find_package(Threads REQUIRED)

add_library(ogun_app_controller STATIC
    AppController.cpp
    AppControllerHoles.cpp
    AppControllerIntersections.cpp
    AppControllerMerge.cpp
    DefectBrowser.cpp
    HealthCheckRunner.cpp
    MergeJobRunner.cpp
    RepairJobRunner.cpp
    ShellBrowser.cpp
)
add_library(ogun::app_controller ALIAS ogun_app_controller)

target_include_directories(ogun_app_controller
    PUBLIC
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_compile_definitions(ogun_app_controller
    PUBLIC
        OGUN_SOURCE_ICON_PNG_PATH="${CMAKE_SOURCE_DIR}/assets/icons/ogun.png"
        OGUN_SOURCE_ICON_JPEG_PATH="${CMAKE_SOURCE_DIR}/assets/icons/ogun.jpeg"
        OGUN_SOURCE_EXAMPLES_DIR="${CMAKE_SOURCE_DIR}/examples"
)

target_link_libraries(ogun_app_controller
    PUBLIC
        ogun::mesh
        ogun::io
        ogun::diagnostics
        ogun::repair
        Threads::Threads
    PRIVATE
        ogun::warnings
)

if(OGUN_BUILD_GUI)
    add_executable(ogun
        main.cpp
        PolyscopeGui.cpp
        PolyscopeGuiShell.cpp
        PolyscopeGuiPanels.cpp
        PolyscopeGuiHealth.cpp
        PolyscopeGuiHoles.cpp
        PolyscopeGuiIntersections.cpp
        PolyscopeGuiShells.cpp
        GuiWidgets.cpp
        MacAppIcon.cpp
    )

    if(APPLE)
        set_source_files_properties(MacAppIcon.cpp PROPERTIES
            COMPILE_OPTIONS "-x;objective-c++"
        )
    endif()

    target_include_directories(ogun
        PRIVATE
            "${CMAKE_SOURCE_DIR}/src"
            "${CMAKE_CURRENT_SOURCE_DIR}"
    )

    target_link_libraries(ogun
        PRIVATE
            ogun::app_controller
            polyscope
            portable_file_dialogs
            ogun::warnings
    )
endif()

if(OGUN_BUILD_TESTS)
    add_subdirectory(Test)
endif()

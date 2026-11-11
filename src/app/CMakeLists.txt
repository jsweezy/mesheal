find_package(Threads REQUIRED)

add_library(mesheal_app_controller STATIC
    AboutInfo.cpp
    AppController.cpp
    AppControllerCleanup.cpp
    AppControllerFixAll.cpp
    AppControllerHoles.cpp
    AppControllerIntersections.cpp
    AppControllerMerge.cpp
    AppControllerOrientation.cpp
    AppControllerSelection.cpp
    AppControllerTransform.cpp
    CliRunner.cpp
    DefectBrowser.cpp
    FaceSelection.cpp
    FileOpenQueue.cpp
    FixAllPanelModel.cpp
    FixAllPlanRunner.cpp
    FramePacer.cpp
    GuiLayout.cpp
    HealthCheckRunner.cpp
    MergeJobRunner.cpp
    MouseMapping.cpp
    RepairJobRunner.cpp
    ShellBrowser.cpp
    RecentFiles.cpp
    UiTheme.cpp
    UserPreferences.cpp
)
add_library(mesheal::app_controller ALIAS mesheal_app_controller)

target_include_directories(mesheal_app_controller
    PUBLIC
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_compile_definitions(mesheal_app_controller
    PUBLIC
        MESHEAL_SOURCE_ICON_PNG_PATH="${CMAKE_SOURCE_DIR}/assets/icons/mesheal.png"
        MESHEAL_SOURCE_ICON_JPEG_PATH="${CMAKE_SOURCE_DIR}/assets/icons/mesheal.jpeg"
        MESHEAL_SOURCE_EXAMPLES_DIR="${CMAKE_SOURCE_DIR}/examples"
)

target_link_libraries(mesheal_app_controller
    PUBLIC
        mesheal::core
        mesheal::mesh
        mesheal::io
        mesheal::diagnostics
        mesheal::repair
        Threads::Threads
    PRIVATE
        mesheal::warnings
)
if(WIN32)
    # ShellExecuteA (openUrlInBrowser)
    target_link_libraries(mesheal_app_controller PRIVATE shell32)
endif()

# Command-line-only executable: no GUI dependencies. It is `mesheal` in a CLI-only build
# (MESHEAL_BUILD_GUI=OFF) and `mesheal-cli` next to the desktop app otherwise (the Docker image uses it).
add_executable(mesheal_cli CliMain.cpp)
target_link_libraries(mesheal_cli
    PRIVATE
        mesheal::app_controller
        mesheal::warnings
)
if(MESHEAL_BUILD_GUI)
    set_target_properties(mesheal_cli PROPERTIES OUTPUT_NAME "mesheal-cli")
else()
    set_target_properties(mesheal_cli PROPERTIES OUTPUT_NAME "mesheal")
    install(TARGETS mesheal_cli RUNTIME DESTINATION bin)
endif()

if(MESHEAL_BUILD_GUI)
    add_executable(mesheal
        main.cpp
        PolyscopeGui.cpp
        PolyscopeGuiShell.cpp
        PolyscopeGuiPanels.cpp
        PolyscopeGuiFixAll.cpp
        PolyscopeGuiHealth.cpp
        PolyscopeGuiHoles.cpp
        PolyscopeGuiIntersections.cpp
        PolyscopeGuiSelection.cpp
        PolyscopeGuiShells.cpp
        GuiTheme.cpp
        GuiWidgets.cpp
        MacAppIcon.cpp
    )

    if(APPLE)
        set(MESHEAL_ICNS_RESOURCE "${CMAKE_SOURCE_DIR}/assets/icons/mesheal.icns")
        set_source_files_properties("${MESHEAL_ICNS_RESOURCE}" PROPERTIES
            MACOSX_PACKAGE_LOCATION "Resources"
        )
        target_sources(mesheal PRIVATE
            "${MESHEAL_ICNS_RESOURCE}"
            MacFileOpenHandler.mm
        )
        set_source_files_properties(MacAppIcon.cpp MacFileOpenHandler.mm PROPERTIES
            COMPILE_OPTIONS "-x;objective-c++"
        )
        set_target_properties(mesheal PROPERTIES
            OUTPUT_NAME "Mesheal"
            MACOSX_BUNDLE TRUE
            MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/Info.plist.in"
            MACOSX_BUNDLE_BUNDLE_NAME "Mesheal"
            MACOSX_BUNDLE_EXECUTABLE_NAME "Mesheal"
            MACOSX_BUNDLE_GUI_IDENTIFIER "io.github.jsweezy.mesheal"
            MACOSX_BUNDLE_ICON_FILE "mesheal.icns"
            MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION}"
            MACOSX_BUNDLE_BUNDLE_VERSION "${PROJECT_VERSION}"
        )
        target_link_libraries(mesheal PRIVATE "-framework Cocoa")

        # Symlink build_dir/mesheal -> build_dir/Mesheal.app/Contents/MacOS/Mesheal for CLI testing & scripts
        add_custom_command(TARGET mesheal POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E create_symlink
                "Mesheal.app/Contents/MacOS/Mesheal"
                "${CMAKE_CURRENT_BINARY_DIR}/mesheal"
            COMMENT "Creating symlink ${CMAKE_CURRENT_BINARY_DIR}/mesheal -> Mesheal.app/Contents/MacOS/Mesheal"
        )

        install(TARGETS mesheal BUNDLE DESTINATION /Applications)
    elseif(UNIX)
        install(TARGETS mesheal RUNTIME DESTINATION bin)
        install(FILES "${CMAKE_SOURCE_DIR}/packaging/linux/mesheal.desktop"
            DESTINATION share/applications
        )
        install(FILES "${CMAKE_SOURCE_DIR}/packaging/linux/mesheal-mime.xml"
            DESTINATION share/mime/packages
            RENAME mesheal.xml
        )
        install(FILES "${CMAKE_SOURCE_DIR}/assets/icons/mesheal.png"
            DESTINATION share/icons/hicolor/512x512/apps
        )
    endif()

    target_include_directories(mesheal
        PRIVATE
            "${CMAKE_SOURCE_DIR}/src"
            "${CMAKE_CURRENT_SOURCE_DIR}"
            # GLFW's header for glfwWaitEventsTimeout (idle frame pacing); the library comes with polyscope
            "$<TARGET_PROPERTY:glfw,INTERFACE_INCLUDE_DIRECTORIES>"
    )

    target_link_libraries(mesheal
        PRIVATE
            mesheal::app_controller
            polyscope
            portable_file_dialogs
            mesheal::warnings
    )
endif()

if(MESHEAL_BUILD_TESTS)
    add_subdirectory(Test)
endif()

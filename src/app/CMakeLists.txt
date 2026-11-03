find_package(Threads REQUIRED)

add_library(ogun_app_controller STATIC
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
    UserPreferences.cpp
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
        ogun::core
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
        PolyscopeGuiFixAll.cpp
        PolyscopeGuiHealth.cpp
        PolyscopeGuiHoles.cpp
        PolyscopeGuiIntersections.cpp
        PolyscopeGuiSelection.cpp
        PolyscopeGuiShells.cpp
        GuiWidgets.cpp
        MacAppIcon.cpp
    )

    if(APPLE)
        set(OGUN_ICNS_RESOURCE "${CMAKE_SOURCE_DIR}/assets/icons/ogun.icns")
        set_source_files_properties("${OGUN_ICNS_RESOURCE}" PROPERTIES
            MACOSX_PACKAGE_LOCATION "Resources"
        )
        target_sources(ogun PRIVATE
            "${OGUN_ICNS_RESOURCE}"
            MacFileOpenHandler.mm
        )
        set_source_files_properties(MacAppIcon.cpp MacFileOpenHandler.mm PROPERTIES
            COMPILE_OPTIONS "-x;objective-c++"
        )
        set_target_properties(ogun PROPERTIES
            MACOSX_BUNDLE TRUE
            MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/Info.plist.in"
            MACOSX_BUNDLE_BUNDLE_NAME "Ogun"
            MACOSX_BUNDLE_EXECUTABLE_NAME "ogun"
            MACOSX_BUNDLE_GUI_IDENTIFIER "io.github.jsweezy.ogun"
            MACOSX_BUNDLE_ICON_FILE "ogun.icns"
            MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION}"
            MACOSX_BUNDLE_BUNDLE_VERSION "${PROJECT_VERSION}"
        )
        target_link_libraries(ogun PRIVATE "-framework Cocoa")

        # Symlink build_dir/ogun -> build_dir/ogun.app/Contents/MacOS/ogun for CLI testing & scripts
        add_custom_command(TARGET ogun POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E create_symlink
                "ogun.app/Contents/MacOS/ogun"
                "${CMAKE_CURRENT_BINARY_DIR}/ogun"
            COMMENT "Creating symlink ${CMAKE_CURRENT_BINARY_DIR}/ogun -> ogun.app/Contents/MacOS/ogun"
        )

        install(TARGETS ogun BUNDLE DESTINATION /Applications)
    elseif(UNIX)
        install(TARGETS ogun RUNTIME DESTINATION bin)
        install(FILES "${CMAKE_SOURCE_DIR}/packaging/linux/ogun.desktop"
            DESTINATION share/applications
        )
        install(FILES "${CMAKE_SOURCE_DIR}/packaging/linux/ogun-mime.xml"
            DESTINATION share/mime/packages
            RENAME ogun.xml
        )
        install(FILES "${CMAKE_SOURCE_DIR}/assets/icons/ogun.png"
            DESTINATION share/icons/hicolor/512x512/apps
        )
    endif()

    target_include_directories(ogun
        PRIVATE
            "${CMAKE_SOURCE_DIR}/src"
            "${CMAKE_CURRENT_SOURCE_DIR}"
            # GLFW's header for glfwWaitEventsTimeout (idle frame pacing); the library comes with polyscope
            "$<TARGET_PROPERTY:glfw,INTERFACE_INCLUDE_DIRECTORIES>"
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

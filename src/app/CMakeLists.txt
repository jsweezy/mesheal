add_library(ogun_app_controller STATIC
    AppController.cpp
)
add_library(ogun::app_controller ALIAS ogun_app_controller)

target_include_directories(ogun_app_controller
    PUBLIC
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(ogun_app_controller
    PUBLIC
        ogun::mesh
        ogun::io
        ogun::diagnostics
    PRIVATE
        ogun::warnings
)

if(OGUN_BUILD_GUI)
    add_executable(ogun_app
        main.cpp
        PolyscopeGui.cpp
    )

    target_include_directories(ogun_app
        PRIVATE
            "${CMAKE_SOURCE_DIR}/src"
            "${CMAKE_CURRENT_SOURCE_DIR}"
    )

    target_link_libraries(ogun_app
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

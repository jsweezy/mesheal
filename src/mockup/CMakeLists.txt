# Throwaway clickable prototype of the GUI layout in UI_DESIGN.md (fake defect data).
add_executable(mesheal_ui_mockup
    main.cpp
    UiMockup.cpp
)

target_include_directories(mesheal_ui_mockup
    PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
)

target_compile_definitions(mesheal_ui_mockup
    PRIVATE
        MESHEAL_MOCKUP_DEFAULT_STL="${CMAKE_SOURCE_DIR}/examples/stanford_bunny.stl"
)

target_link_libraries(mesheal_ui_mockup
    PRIVATE
        mesheal::io
        mesheal::diagnostics
        polyscope
        mesheal::warnings
)

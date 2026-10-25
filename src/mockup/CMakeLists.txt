# Throwaway clickable prototype of the GUI layout in UI_DESIGN.md (fake defect data).
add_executable(ogun_ui_mockup
    main.cpp
    UiMockup.cpp
)

target_include_directories(ogun_ui_mockup
    PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
)

target_compile_definitions(ogun_ui_mockup
    PRIVATE
        OGUN_MOCKUP_DEFAULT_STL="${CMAKE_SOURCE_DIR}/examples/stanford_bunny.stl"
)

target_link_libraries(ogun_ui_mockup
    PRIVATE
        ogun::io
        ogun::diagnostics
        polyscope
        ogun::warnings
)

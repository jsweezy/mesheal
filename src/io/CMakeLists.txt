add_library(ogun_io STATIC
    StlReader.cpp
    StlWriter.cpp
    SampleMeshes.cpp
)
add_library(ogun::io ALIAS ogun_io)

target_include_directories(ogun_io
    PUBLIC
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(ogun_io
    PUBLIC
        ogun::mesh
        igl::core
    PRIVATE
        ogun::diagnostics
        ogun::warnings
)

if(OGUN_BUILD_TESTS)
    add_subdirectory(Test)
endif()

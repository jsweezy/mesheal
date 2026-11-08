add_library(mesheal_io STATIC
    ConfigDirectory.cpp
    StlReader.cpp
    StlWriter.cpp
    SampleMeshes.cpp
)
add_library(mesheal::io ALIAS mesheal_io)

target_include_directories(mesheal_io
    PUBLIC
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(mesheal_io
    PUBLIC
        mesheal::mesh
        igl::core
    PRIVATE
        mesheal::diagnostics
        mesheal::warnings
)

if(MESHEAL_BUILD_TESTS)
    add_subdirectory(Test)
endif()

add_library(mesheal_diagnostics STATIC
    ThinFaceChecker.cpp
    MeshHealth.cpp
    MeshChecks.cpp
    TriTriIntersect.cpp
    SelfIntersections.cpp
    Histogram.cpp
    HoleLoops.cpp
    ShellNesting.cpp
    ShellRelations.cpp
    ShellRoles.cpp
)
add_library(mesheal::diagnostics ALIAS mesheal_diagnostics)

target_include_directories(mesheal_diagnostics
    PUBLIC
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(mesheal_diagnostics
    PUBLIC
        mesheal::mesh
        igl::core
    PRIVATE
        mesheal::warnings
)

if(MESHEAL_BUILD_TESTS)
    add_subdirectory(Test)
endif()

add_library(ogun_diagnostics STATIC
    ThinFaceChecker.cpp
    MeshHealth.cpp
    MeshChecks.cpp
    TriTriIntersect.cpp
    SelfIntersections.cpp
    Histogram.cpp
)
add_library(ogun::diagnostics ALIAS ogun_diagnostics)

target_include_directories(ogun_diagnostics
    PUBLIC
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(ogun_diagnostics
    PUBLIC
        ogun::mesh
        igl::core
    PRIVATE
        ogun::warnings
)

if(OGUN_BUILD_TESTS)
    add_subdirectory(Test)
endif()

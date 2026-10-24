add_library(ogun_mesh STATIC
    CgalMesh.cpp
    Mesh.cpp
    Units.cpp
)
add_library(ogun::mesh ALIAS ogun_mesh)

target_include_directories(ogun_mesh
    PUBLIC
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(ogun_mesh
    PUBLIC
        Eigen3::Eigen
        igl::core
        manifold::manifold
        CGAL::CGAL
    PRIVATE
        ogun::warnings
)

if(OGUN_BUILD_TESTS)
    add_subdirectory(Test)
endif()

add_library(mesheal_mesh STATIC
    CgalMesh.cpp
    Mesh.cpp
    MeshTransform.cpp
    Units.cpp
)
add_library(mesheal::mesh ALIAS mesheal_mesh)

target_include_directories(mesheal_mesh
    PUBLIC
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(mesheal_mesh
    PUBLIC
        Eigen3::Eigen
        igl::core
        manifold::manifold
        CGAL::CGAL
    PRIVATE
        mesheal::warnings
)

if(MESHEAL_BUILD_TESTS)
    add_subdirectory(Test)
endif()

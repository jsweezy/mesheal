add_executable(mesheal_mesh_tests
    MeshAndUnitsTests.cpp
    MeshTransformTests.cpp
)

target_link_libraries(mesheal_mesh_tests
    PRIVATE
        mesheal::mesh
        Catch2::Catch2WithMain
        mesheal::warnings
)

catch_discover_tests(mesheal_mesh_tests)

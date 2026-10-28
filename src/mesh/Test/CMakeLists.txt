add_executable(ogun_mesh_tests
    MeshAndUnitsTests.cpp
    MeshTransformTests.cpp
)

target_link_libraries(ogun_mesh_tests
    PRIVATE
        ogun::mesh
        Catch2::Catch2WithMain
        ogun::warnings
)

catch_discover_tests(ogun_mesh_tests)

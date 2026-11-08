# Python unit tests for the _mesheal module (numpy only, synthetic meshes).
add_test(
    NAME mesheal_python_tests
    COMMAND ${Python_EXECUTABLE} -m unittest -v test_mesheal
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
set_tests_properties(mesheal_python_tests PROPERTIES
    ENVIRONMENT "PYTHONPATH=$<TARGET_FILE_DIR:_mesheal>"
    TIMEOUT 180
)

# Headless Blender smoke test of the extension. It imports the module copied into
# blender/mesheal_tools by scripts/build-blender-module.sh, so run that script first.
find_program(BLENDER_EXECUTABLE blender PATHS /Applications/Blender.app/Contents/MacOS)
if(BLENDER_EXECUTABLE)
    add_test(
        NAME mesheal_blender_smoke
        COMMAND ${BLENDER_EXECUTABLE} --background --factory-startup
                --python ${CMAKE_SOURCE_DIR}/blender/tests/smoke_test.py
    )
    set_tests_properties(mesheal_blender_smoke PROPERTIES TIMEOUT 300)
endif()

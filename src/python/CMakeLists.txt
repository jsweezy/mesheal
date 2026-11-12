nanobind_add_module(_mesheal NB_STATIC
    Module.cpp
    Params.cpp
    Convert.cpp
    Job.cpp
)

# nanobind's headers are not warning-clean under our flags: treat them as system headers.
target_include_directories(_mesheal SYSTEM PRIVATE "${nanobind_SOURCE_DIR}/include")

target_link_libraries(_mesheal
    PRIVATE
        mesheal::repair
        mesheal::core
        mesheal::warnings
)

if(MESHEAL_BLENDER_TBB_LIBRARY)
    # Find Blender's TBB at run time, never the copy linked against at build time: on Linux and
    # Windows Blender has already loaded it (found by name); on macOS it is in Blender.app's
    # Contents/Resources/lib, and @executable_path is Blender itself.
    set_target_properties(_mesheal PROPERTIES BUILD_WITH_INSTALL_RPATH ON)
    if(APPLE)
        set_target_properties(_mesheal PROPERTIES INSTALL_RPATH "@executable_path/../Resources/lib")
    endif()
endif()

if(MESHEAL_BUILD_TESTS)
    add_subdirectory(Test)
endif()

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

if(MESHEAL_BUILD_TESTS)
    add_subdirectory(Test)
endif()

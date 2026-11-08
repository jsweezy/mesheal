add_library(mesheal_core INTERFACE)
add_library(mesheal::core ALIAS mesheal_core)

target_link_libraries(mesheal_core INTERFACE mesheal::version)

target_include_directories(mesheal_core
    INTERFACE
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

if(MESHEAL_BUILD_TESTS)
    add_subdirectory(Test)
endif()

add_library(ogun_core INTERFACE)
add_library(ogun::core ALIAS ogun_core)

target_link_libraries(ogun_core INTERFACE ogun::version)

target_include_directories(ogun_core
    INTERFACE
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

if(OGUN_BUILD_TESTS)
    add_subdirectory(Test)
endif()

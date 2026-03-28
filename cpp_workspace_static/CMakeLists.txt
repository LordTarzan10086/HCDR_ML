cmake_minimum_required(VERSION 3.20)

project(cpp_workspace_static LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT DEFINED HCDR_PIN_ENV)
    set(HCDR_PIN_ENV "C:/Users/A/miniconda3/envs/hcdr_pin")
endif()

list(APPEND CMAKE_PREFIX_PATH "${HCDR_PIN_ENV}" "${HCDR_PIN_ENV}/Library")

find_package(pinocchio CONFIG REQUIRED)
find_package(osqp CONFIG REQUIRED)

set(WORKSPACE_STATIC_SOURCES
    src/workspace_config.cpp
    src/platform_geometry.cpp
    src/arm_pinocchio_model.cpp
    src/platform_static_feasibility.cpp
    src/point_cloud_io.cpp
    src/workspace_analyzer.cpp
)

add_library(workspace_static_lib STATIC ${WORKSPACE_STATIC_SOURCES})
target_include_directories(workspace_static_lib PUBLIC include)
target_compile_definitions(
    workspace_static_lib
    PUBLIC WORKSPACE_STATIC_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(
    workspace_static_lib
    PUBLIC
        osqp::osqp
        pinocchio::pinocchio_default
        pinocchio::pinocchio_parsers
)
target_compile_definitions(workspace_static_lib PUBLIC NOMINMAX)

add_executable(workspace_static_runner app/main.cpp)
target_link_libraries(workspace_static_runner PRIVATE workspace_static_lib)

add_executable(workspace_static_tests tests/test_main.cpp)
target_link_libraries(workspace_static_tests PRIVATE workspace_static_lib)

enable_testing()
add_test(NAME workspace_static_tests COMMAND workspace_static_tests)
if(WIN32)
    set(HCDR_TEST_PATH "${HCDR_PIN_ENV}/Library/bin")
    set_tests_properties(
        workspace_static_tests
        PROPERTIES ENVIRONMENT "PATH=${HCDR_TEST_PATH};$ENV{PATH}"
    )
endif()

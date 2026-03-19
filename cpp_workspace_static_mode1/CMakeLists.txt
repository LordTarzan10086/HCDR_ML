cmake_minimum_required(VERSION 3.20)

project(cpp_workspace_static_mode1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT DEFINED HCDR_PIN_ENV)
    set(HCDR_PIN_ENV "C:/Users/A/miniconda3/envs/hcdr_pin")
endif()

list(APPEND CMAKE_PREFIX_PATH "${HCDR_PIN_ENV}" "${HCDR_PIN_ENV}/Library")

find_package(pinocchio CONFIG REQUIRED)
find_package(osqp CONFIG REQUIRED)

set(MODE1_SOURCES
    src/workspace_config.cpp
    src/platform_geometry.cpp
    src/arm_pinocchio_model.cpp
    src/platform_static_feasibility.cpp
    src/pointcloud_export.cpp
    src/plot_csv_export.cpp
    src/mode1_workspace_solver.cpp
)

add_library(workspace_static_mode1_lib STATIC ${MODE1_SOURCES})
target_include_directories(workspace_static_mode1_lib PUBLIC src)
target_compile_definitions(
    workspace_static_mode1_lib
    PUBLIC WORKSPACE_STATIC_MODE1_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(
    workspace_static_mode1_lib
    PUBLIC
        osqp::osqp
        pinocchio::pinocchio_default
        pinocchio::pinocchio_parsers
)
target_compile_definitions(workspace_static_mode1_lib PUBLIC NOMINMAX)

if(WIN32)
    target_link_libraries(workspace_static_mode1_lib PUBLIC gdiplus)
endif()

add_executable(mode1_workspace_runner app/main_mode1_workspace.cpp)
target_link_libraries(mode1_workspace_runner PRIVATE workspace_static_mode1_lib)

add_executable(mode1_workspace_tests tests/test_main.cpp)
target_link_libraries(mode1_workspace_tests PRIVATE workspace_static_mode1_lib)

enable_testing()
add_test(NAME mode1_workspace_tests COMMAND mode1_workspace_tests)
if(WIN32)
    set(HCDR_TEST_PATH "${HCDR_PIN_ENV}/Library/bin")
    set_tests_properties(
        mode1_workspace_tests
        PROPERTIES ENVIRONMENT "PATH=${HCDR_TEST_PATH};$ENV{PATH}"
    )
endif()

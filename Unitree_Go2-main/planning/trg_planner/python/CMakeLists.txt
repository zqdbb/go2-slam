cmake_minimum_required(VERSION 3.16...3.24)
project(trg_planner_pybind VERSION 1.0.0)

# Set build type
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

message(STATUS "Python Interpreter Version: ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")

include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11
  GIT_TAG master
)
FetchContent_MakeAvailable(pybind11)

set(PYBIND11_NEWPYTHON ON)
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)

message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "CMAKE_CURRENT_BINARY_DIR: ${CMAKE_CURRENT_BINARY_DIR}")

if (DEFINED SKBUILD)
    message(STATUS "Building with Scikit-Build")
endif ()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../cpp/trg_planner/)
  message(STATUS "Install TRG-planner using local C++ files...")
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp/trg_planner ${CMAKE_CURRENT_BINARY_DIR}/trg_planner)
else()
  cmake_minimum_required(VERSION 3.16...3.24)
  message(STATUS "Performing out-of-tree build, fetching TRG-planner v${CMAKE_PROJECT_VERSION} Release from Github")
  include(FetchContent)
  FetchContent_Declare(
    ext_trg_planner_core PREFIX trg_planner
    GIT_REPOSITORY https://github.com/url-kaist/TRG-planner.git
    GIT_TAG main
    SOURCE_SUBDIR cpp/trg_planner)
  FetchContent_MakeAvailable(ext_trg_planner_core)
endif()

set(OUTPUT_MODULE_NAME trg_planner)

pybind11_add_module(${OUTPUT_MODULE_NAME} trg_planner/pybind/trg_planner_pybind.cpp)
target_link_libraries(${OUTPUT_MODULE_NAME} PUBLIC trg_planner_core)

install(TARGETS ${OUTPUT_MODULE_NAME} DESTINATION .)

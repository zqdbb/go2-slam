cmake_minimum_required(VERSION 3.16...3.24)
project(trg_planner_example VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

include(FetchContent)

# set package directory path
set(TRG_EXAMPLE_DIR ${CMAKE_CURRENT_LIST_DIR})
message(STATUS "TRG_EXAMPLE_DIR: ${TRG_EXAMPLE_DIR}")
add_definitions(-DTRG_EXAMPLE_DIR="${TRG_EXAMPLE_DIR}")

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(trg_planner REQUIRED)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/include
)

add_executable(run_trg_planner
    ${CMAKE_CURRENT_LIST_DIR}/src/run_trg_planner.cc
)
target_include_directories(run_trg_planner
    PUBLIC
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(run_trg_planner
    trg_planner::trg_planner_core
    ${PCL_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${OpenCV_LIBRARIES}
    yaml-cpp
    pthread
)

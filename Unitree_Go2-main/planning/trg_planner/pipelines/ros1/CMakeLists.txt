cmake_minimum_required(VERSION 3.16...3.24)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

message(STATUS "Configuring pipelines/ros1")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    pcl_conversions
    pcl_ros
    grid_map_ros
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(trg_planner REQUIRED)

catkin_package()

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${catkin_INCLUDE_DIRS}
)

add_executable(trg_ros1_node
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/ros1_node.cpp
)
target_link_libraries(trg_ros1_node
    ${catkin_LIBRARIES}
    trg_planner::trg_planner_core
)

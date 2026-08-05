cmake_minimum_required(VERSION 3.16...3.24)
project(trg_planner_ros VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)

# Check build types
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE
            STRING "Choose the type of build." FORCE)
endif ()

# set package directory path
set(TRG_ROS_DIR ${CMAKE_CURRENT_LIST_DIR})
add_definitions(-DTRG_ROS_DIR="${TRG_ROS_DIR}")
message(STATUS "TRG_ROS_DIR: ${TRG_ROS_DIR}")


# ROS 1
if (DEFINED CATKIN_DEVEL_PREFIX)
    message(STATUS "ROS1 build")

    find_package(catkin REQUIRED COMPONENTS roscpp)

    add_subdirectory(ros1)

# ROS 2
elseif (DEFINED ENV{AMENT_PREFIX_PATH})
    message(STATUS "ROS2 build")

    find_package(rclcpp REQUIRED)

    add_subdirectory(ros2)
endif()

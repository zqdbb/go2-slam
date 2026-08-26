cmake_minimum_required(VERSION 3.8)
project(world_tf_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(world_tf_publisher SHARED src/world_tf_publisher.cpp)
target_include_directories(world_tf_publisher PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(world_tf_publisher ${GAZEBO_LIBRARIES})

ament_target_dependencies(world_tf_publisher
  gazebo_ros
  rclcpp
  tf2_ros
  geometry_msgs
)

install(TARGETS world_tf_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
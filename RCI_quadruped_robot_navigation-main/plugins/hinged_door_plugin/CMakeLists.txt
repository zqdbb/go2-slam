cmake_minimum_required(VERSION 3.8)
project(hinged_door_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)          # Gazebo Classic
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# === Double hinged door plugin ===
add_library(double_hinged_door_plugin SHARED src/double_hinged_door_plugin.cpp)

target_include_directories(double_hinged_door_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})

target_link_libraries(double_hinged_door_plugin ${GAZEBO_LIBRARIES})

ament_target_dependencies(double_hinged_door_plugin gazebo_ros rclcpp std_msgs)


# === Single hinged door plugin ===
add_library(single_hinged_door_plugin SHARED src/single_hinged_door_plugin.cpp)

target_include_directories(single_hinged_door_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})

target_link_libraries(single_hinged_door_plugin ${GAZEBO_LIBRARIES})

ament_target_dependencies(single_hinged_door_plugin gazebo_ros rclcpp std_msgs)

install(TARGETS
  double_hinged_door_plugin
  single_hinged_door_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
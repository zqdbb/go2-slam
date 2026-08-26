cmake_minimum_required(VERSION 3.8)
project(sliding_door_plugin)

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

# === Double door plugin ===
add_library(double_sliding_door_plugin SHARED src/double_sliding_door_plugin.cpp)
            
target_include_directories(double_sliding_door_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})

target_link_libraries(double_sliding_door_plugin ${GAZEBO_LIBRARIES})

ament_target_dependencies(double_sliding_door_plugin gazebo_ros rclcpp std_msgs)


# === Single door plugin ===
add_library(single_sliding_door_plugin SHARED src/single_sliding_door_plugin.cpp)

target_include_directories(single_sliding_door_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})

target_link_libraries(single_sliding_door_plugin ${GAZEBO_LIBRARIES})

ament_target_dependencies(single_sliding_door_plugin gazebo_ros rclcpp std_msgs)

install(TARGETS
  double_sliding_door_plugin
  single_sliding_door_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
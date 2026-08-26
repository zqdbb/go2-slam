cmake_minimum_required(VERSION 3.8)
project(door_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
)
add_executable(door_bt_runner
  src/door_bt_runner.cpp
)

add_library(door_bt_nodes SHARED
  src/bt_nodes/bt_plugin.cpp
  src/bt_nodes/door_plan_controller.cpp
)

ament_target_dependencies(door_bt_nodes
  rclcpp
  nav_msgs
  behaviortree_cpp_v3
  tf2_ros                                  
  geometry_msgs     
  std_msgs
  ament_index_cpp
)

target_link_libraries(door_bt_runner
  door_bt_nodes
)

install(
  TARGETS door_bt_nodes
  LIBRARY DESTINATION lib
)

install(
  TARGETS door_bt_runner
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY bt_trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
if(TARGET yaml-cpp)
  target_link_libraries(door_bt_nodes yaml-cpp)
else()  
  find_package(YAML-CPP REQUIRED)
  target_link_libraries(door_bt_nodes ${YAML_CPP_LIBRARIES})
endif()
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
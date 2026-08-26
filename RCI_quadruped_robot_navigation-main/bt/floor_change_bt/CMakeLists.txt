cmake_minimum_required(VERSION 3.8)
project(floor_change_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED) 
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(floor_change_bt_runner
  src/floor_change_bt_runner.cpp
  src/bt_nodes/get_current_floor.cpp
  src/bt_nodes/publish_door_yaml.cpp
  src/bt_nodes/publish_initial_pose_from_tf.cpp
  src/bt_nodes/select_floor_config.cpp

)

ament_target_dependencies(floor_change_bt_runner
  rclcpp
  behaviortree_cpp_v3
  ament_index_cpp
  std_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
)

install(TARGETS floor_change_bt_runner
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY bt_trees
  DESTINATION share/${PROJECT_NAME}
)

target_include_directories(floor_change_bt_runner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(floor_change_bt_runner
  yaml-cpp
)


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

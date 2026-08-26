cmake_minimum_required(VERSION 3.8)
project(stair_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)    
find_package(ament_index_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)


include_directories(
  include
)

add_library(stair_bt_nav2 SHARED
  src/bt_nodes/turn_relative.cpp
  src/bt_nodes/align_with_stairs.cpp
  src/bt_nodes/move_forward_distance.cpp
  src/bt_nodes/climb_flight_rl.cpp
  src/bt_nodes/get_target_floor_from_topic.cpp
  src/bt_nodes/check_floor_different.cpp
  src/bt_nodes/select_stair_entrance.cpp
  src/stair_bt_nav2_plugin.cpp
  src/bt_nodes/nav2_navigate_to_pose.cpp  
  src/bt_nodes/compute_next_floor.cpp
)

ament_target_dependencies(stair_bt_nav2
  rclcpp
  behaviortree_cpp_v3
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
  nav2_msgs
)

target_compile_definitions(stair_bt_nav2 PRIVATE BT_PLUGIN_EXPORT)

install(
  DIRECTORY include/
  DESTINATION include
)


add_executable(stair_bt_runner
  src/stair_bt_runner.cpp
)

ament_target_dependencies(stair_bt_runner
  rclcpp
  behaviortree_cpp_v3
  nav2_msgs
)
install(
  TARGETS stair_bt_nav2 stair_bt_runner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} 
)


install(
  DIRECTORY bt_trees
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(TARGET yaml-cpp)
  target_link_libraries(stair_bt_runner yaml-cpp)
else()
  find_package(YAML-CPP REQUIRED)
  target_link_libraries(stair_bt_runner ${YAML_CPP_LIBRARIES})
endif()

if(TARGET yaml-cpp)
  target_link_libraries(stair_bt_nav2 yaml-cpp)
else()
  find_package(YAML-CPP REQUIRED)
  target_link_libraries(stair_bt_nav2 ${YAML_CPP_LIBRARIES})
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

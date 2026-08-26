cmake_minimum_required(VERSION 3.8)
project(elevator_bt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)    
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED) 
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(elevator_bt_runner
  src/main_bt.cpp
  src/bt_nodes/ok_node.cpp
  src/bt_nodes/call_elevator_to_floor.cpp
  src/bt_nodes/set_elevator_door.cpp
  src/bt_nodes/wait_door_open.cpp
  src/bt_nodes/wait_cabin_at_target.cpp
  src/bt_nodes/wait_robot_near_elevator.cpp
  src/bt_nodes/wait_robot_inside_cabin.cpp
  src/bt_nodes/wait_robot_outside_elevator.cpp
  src/bt_nodes/elevator_client.cpp
  src/bt_nodes/select_nearest_elevator.cpp
  src/bt_nodes/get_cabin_goal.cpp
  src/bt_nodes/wait_floor_command.cpp 
  src/bt_nodes/nav2_navigate_to_pose.cpp  
  )

target_include_directories(elevator_bt_runner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(elevator_bt_runner
  rclcpp
  behaviortree_cpp_v3
  geometry_msgs
  tf2
  tf2_ros
  std_msgs
  rclcpp_action 
  nav2_msgs
)

if(TARGET yaml-cpp)
  target_link_libraries(elevator_bt_runner yaml-cpp)
else()  
  find_package(YAML-CPP REQUIRED)
  target_link_libraries(elevator_bt_runner ${YAML_CPP_LIBRARIES})
endif()

install(TARGETS elevator_bt_runner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config/
  DESTINATION config)

install(DIRECTORY bt_trees/
  DESTINATION share/${PROJECT_NAME}/bt_trees)

  add_library(elevator_bt_nav2 SHARED
  src/bt_nodes/ok_node.cpp
  src/bt_nodes/call_elevator_to_floor.cpp
  src/bt_nodes/set_elevator_door.cpp
  src/bt_nodes/wait_door_open.cpp
  src/bt_nodes/wait_cabin_at_target.cpp
  src/bt_nodes/wait_robot_near_elevator.cpp
  src/bt_nodes/wait_robot_inside_cabin.cpp
  src/bt_nodes/wait_robot_outside_elevator.cpp
  src/bt_nodes/elevator_client.cpp
  src/bt_nodes/select_nearest_elevator.cpp
  src/bt_nodes/get_cabin_goal.cpp
  src/bt_nodes/wait_floor_command.cpp 
  src/bt_nodes/elevator_bt_nav2_plugin.cpp    
  src/bt_nodes/nav2_navigate_to_pose.cpp  

)

target_include_directories(elevator_bt_nav2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(elevator_bt_nav2 PRIVATE BT_PLUGIN_EXPORT)

ament_target_dependencies(elevator_bt_nav2
  rclcpp
  behaviortree_cpp_v3
  geometry_msgs
  tf2
  tf2_ros
  std_msgs
  nav2_msgs
)

if(TARGET yaml-cpp)
  target_link_libraries(elevator_bt_nav2 yaml-cpp)
else()
  find_package(YAML-CPP REQUIRED)
  target_link_libraries(elevator_bt_nav2 ${YAML_CPP_LIBRARIES})
endif()

install(
  TARGETS elevator_bt_nav2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_package()

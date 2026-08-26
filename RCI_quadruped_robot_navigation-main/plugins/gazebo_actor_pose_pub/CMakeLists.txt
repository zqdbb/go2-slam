cmake_minimum_required(VERSION 3.8)
project(gazebo_actor_pose_pub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

add_library(gazebo_ros_actor_pose_pub SHARED src/gazebo_ros_actor_pose_pub.cpp)

target_include_directories(gazebo_ros_actor_pose_pub PRIVATE ${GAZEBO_INCLUDE_DIRS})

target_link_libraries(gazebo_ros_actor_pose_pub ${GAZEBO_LIBRARIES})

ament_target_dependencies(gazebo_ros_actor_pose_pub gazebo_ros rclcpp geometry_msgs)


install(TARGETS gazebo_ros_actor_pose_pub
  LIBRARY DESTINATION lib
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

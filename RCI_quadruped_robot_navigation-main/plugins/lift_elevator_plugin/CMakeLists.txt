cmake_minimum_required(VERSION 3.8)
project(lift_elevator_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)  

add_library(elevator_plugin SHARED src/elevator_plugin.cpp)
target_compile_features(elevator_plugin PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(elevator_plugin rclcpp std_msgs gazebo_ros)

# Gazebo Classic include/libs
# Gazebo Classic include/libs
target_include_directories(elevator_plugin PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(elevator_plugin ${GAZEBO_LIBRARIES})

install(TARGETS elevator_plugin
  LIBRARY DESTINATION lib)

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

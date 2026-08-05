#include <rclcpp/rclcpp.hpp>

#include "ros2_node.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  signal(SIGINT, signal_handler);  // to exit program when ctrl+c

  auto     node = std::make_shared<rclcpp::Node>("trg_ros2_node");
  ROS2Node trg_ros2_node(node);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

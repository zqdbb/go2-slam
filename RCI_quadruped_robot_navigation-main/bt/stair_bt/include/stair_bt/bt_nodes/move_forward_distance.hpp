#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace stair_bt
{

class MoveForwardDistance : public BT::StatefulActionNode
{
public:
  MoveForwardDistance(const std::string & name,
                      const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("distance", "이동 거리 (m)"),
      BT::InputPort<double>("linear_speed", "전진 속도 (m/s)"),
      BT::InputPort<std::string>("odom_frame", "기준 odom frame (기본 odom)"),
      BT::InputPort<std::string>("base_frame", "로봇 base frame (기본 base_link)")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};

  double target_distance_{0.0};
  double linear_speed_{0.2};

  double start_x_{0.0};
  double start_y_{0.0};
  bool started_{false};
  rclcpp::Time start_time_;
  double timeout_sec_{20.0};
};

}  // namespace stair_bt

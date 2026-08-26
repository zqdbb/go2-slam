#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/twist.hpp>

namespace stair_bt
{

class AlignWithStairs : public BT::StatefulActionNode
{
public:
  AlignWithStairs(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("stair_yaml"),
      BT::InputPort<std::string>("stair_id"),
      BT::InputPort<std::string>("direction", "up"),

      BT::InputPort<std::string>("world_frame", "map"),
      BT::InputPort<std::string>("base_frame", "base_link"),

    BT::InputPort<double>("yaw_tolerance", 0.15, "yaw tolerance [rad]"),
    BT::InputPort<double>("angular_speed", 0.5, "angular speed [rad/s]"),
    BT::InputPort<double>("timeout", 0.0, "timeout [s], 0=inf"),
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

  bool started_{false};
  double target_yaw_{0.0};

  double yaw_tolerance_{0.15};
  double angular_speed_{0.5};
  double timeout_{0.0};

  std::string world_frame_{"map"};
  std::string base_frame_{"base_link"};

  rclcpp::Time start_time_;
};

}  // namespace stair_bt

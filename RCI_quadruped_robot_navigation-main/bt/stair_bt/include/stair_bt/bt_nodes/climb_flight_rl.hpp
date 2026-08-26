#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/twist.hpp>

namespace stair_bt
{

class ClimbFlightRL : public BT::StatefulActionNode
{
public:
  ClimbFlightRL(const std::string & name,
                const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("direction"),
      BT::InputPort<std::string>("target_level"),   
      BT::InputPort<double>("timeout"),

      BT::InputPort<double>("linear_speed"),
      BT::InputPort<double>("linear_speed_up"),
      BT::InputPort<double>("linear_speed_down"),

      BT::InputPort<double>("dz_from_entry"),

      BT::InputPort<std::string>("world_frame"),
      BT::InputPort<std::string>("base_frame"),

      BT::BidirectionalPort<double>("reference_z"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string world_frame_;
  std::string base_frame_;

  double timeout_sec_;
  double linear_speed_;
  int    direction_sign_;

  std::string target_level_;
  double target_dz_;
  double z_tolerance_;

  double reference_z_;
  bool   have_reference_z_;

  rclcpp::Time start_time_;
  bool started_;
};

}  // namespace stair_bt

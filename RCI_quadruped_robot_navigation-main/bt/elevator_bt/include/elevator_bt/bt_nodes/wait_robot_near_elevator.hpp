#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <atomic>
#include "elevator_bt/bt_nodes/elevator_zones.hpp"

namespace elevator_bt {

class WaitRobotNearElevator : public BT::StatefulActionNode
{
public:
  WaitRobotNearElevator(const std::string& name,
                        const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
          "elevator_yaml", "path to elevator yaml"),
      BT::InputPort<std::string>(
          "world_frame", std::string("world"), "world frame id"),
      BT::InputPort<std::string>(
          "base_frame",  std::string("base_link"), "base frame id"),
      BT::InputPort<double>(
          "timeout", 60.0, "timeout in seconds")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  ElevatorZonesConfig zones_;
  std::string world_frame_{"world"};
  std::string base_frame_{"base_link"};
  double timeout_{60.0};

  std::chrono::steady_clock::time_point t0_;
  std::chrono::steady_clock::time_point last_log_;
};

} // namespace elevator_bt

#pragma once

#include <atomic>
#include <string>

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

namespace elevator_bt {

class WaitFloorCommand : public BT::StatefulActionNode
{
public:
  WaitFloorCommand(const std::string& name,
                   const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name", "/elevator/floor_request",
                                 "topic for floor command"),
      BT::OutputPort<int>("target_floor", "requested target floor")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void ensureSubscription();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;

  std::string topic_name_;
  std::atomic<bool> got_msg_{false};
  std::atomic<int> floor_{0};
};

} // namespace elevator_bt

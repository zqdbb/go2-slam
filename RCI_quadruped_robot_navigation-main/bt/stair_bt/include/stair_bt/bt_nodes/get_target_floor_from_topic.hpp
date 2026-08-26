#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

namespace stair_bt
{

class GetTargetFloorFromTopic : public BT::StatefulActionNode
{
public:
  GetTargetFloorFromTopic(const std::string & name,
                          const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<int>("target_floor", "요청된 목적 층 인덱스"),
      BT::InputPort<std::string>("topic_name", "/stairs/floor_request"),
      BT::InputPort<double>("timeout", "대기 타임아웃 (초, 0이면 무한대기)")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void floorCallback(const std_msgs::msg::Int32::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;

  std::string topic_name_{"/stairs/floor_request"};
  double timeout_sec_{0.0};  
  bool have_msg_{false};
  int last_floor_{0};
  rclcpp::Time start_time_;
};

}  // namespace stair_bt

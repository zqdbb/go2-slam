#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>

namespace floor_change_bt
{

class PublishDoorYaml : public BT::SyncActionNode
{
public:
  PublishDoorYaml(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  std::string current_topic_;

  std::string last_yaml_;

  void ensurePublisher(const std::string& topic);
  rclcpp::Node::SharedPtr getNodeFromBlackboardOrCreate();
};

}  // namespace door_bt

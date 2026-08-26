#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "elevator_bt/bt_nodes/elevator_client.hpp"
#include <yaml-cpp/yaml.h>

namespace elevator_bt {

class SetElevatorDoor : public BT::SyncActionNode {
public:
  SetElevatorDoor(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config),
    node_(std::make_shared<rclcpp::Node>("set_elevator_door")),
    client_(node_) {}

  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<std::string>("elevator_yaml"),
      BT::InputPort<std::string>("elevator_ns"),
      BT::InputPort<bool>("open")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  ElevatorClient client_;
};

} // namespace elevator_bt

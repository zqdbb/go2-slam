#pragma once

#include <behaviortree_cpp_v3/condition_node.h>

namespace floor_change_bt
{

class CheckStringEquals : public BT::ConditionNode
{
public:
  CheckStringEquals(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("key"),
      BT::InputPort<std::string>("value")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string key, value;
    if (!getInput("key", key) || !getInput("value", value)) {
      throw BT::RuntimeError("Missing ports: key/value");
    }
    return (key == value) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

} // namespace floor_change_bt

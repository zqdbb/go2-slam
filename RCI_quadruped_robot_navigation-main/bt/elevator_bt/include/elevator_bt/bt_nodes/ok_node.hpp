#pragma once
#include <behaviortree_cpp_v3/action_node.h>

namespace elevator_bt {

class Ok : public BT::SyncActionNode {
public:
  Ok(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override {
    return BT::NodeStatus::SUCCESS;
  }
};

} // namespace elevator_bt

#pragma once

#include <behaviortree_cpp_v3/action_node.h>

namespace stair_bt
{

class CheckFloorDifferent : public BT::SyncActionNode
{
public:
  CheckFloorDifferent(const std::string & name,
                      const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("current_floor", "현재 층 인덱스"),
      BT::InputPort<int>("target_floor", "목적 층 인덱스"),
      BT::OutputPort<int>("floors_to_climb", "이동해야 할 층 수(|Δ|)"),
      BT::OutputPort<std::string>("direction", "up / down / none")
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace stair_bt

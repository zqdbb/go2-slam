#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <string>

namespace floor_change_bt
{

class SelectFloorConfig : public BT::SyncActionNode
{
public:
  SelectFloorConfig(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  static bool is_absolute(const std::string& p);
  static std::string resolve_path_relative_to_config(const std::string& config_file,
                                                     const std::string& maybe_rel);
};

}  // namespace floor_change_bt

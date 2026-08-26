#include "elevator_bt/bt_nodes/set_elevator_door.hpp"
#include <yaml-cpp/yaml.h>

namespace elevator_bt {

BT::NodeStatus SetElevatorDoor::tick()
{
  std::string yaml_path, ns;
  bool open=false;

  if(!getInput<std::string>("elevator_yaml", yaml_path)){
    RCLCPP_ERROR(node_->get_logger(), "[SetElevatorDoor] Missing port: elevator_yaml");
    return BT::NodeStatus::FAILURE;
  }
  getInput<std::string>("elevator_ns", ns);
  if(!getInput<bool>("open", open)){
    RCLCPP_ERROR(node_->get_logger(), "[SetElevatorDoor] Missing port: open");
    return BT::NodeStatus::FAILURE;
  }

  try {
    auto y = YAML::LoadFile(yaml_path);
    auto e = y["elevator"];
    std::string cfg_ns = e["namespace"].as<std::string>();
    if(!ns.empty()) cfg_ns = ns;
    client_.setNamespace(cfg_ns);
  } catch(const std::exception& ex) {
    RCLCPP_ERROR(node_->get_logger(), "[SetElevatorDoor] YAML read fail: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  client_.setDoorOpen(open);
  RCLCPP_INFO(node_->get_logger(), "[SetElevatorDoor] door -> %s", open ? "OPEN" : "CLOSE");
  return BT::NodeStatus::SUCCESS;
}

} // namespace elevator_bt

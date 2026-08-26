#include "floor_change_bt/bt_nodes/select_floor_config.hpp"

#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

namespace floor_change_bt
{

SelectFloorConfig::SelectFloorConfig(const std::string& name,
                                     const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{}

BT::PortsList SelectFloorConfig::providedPorts()
{
  return {
    // inputs
    BT::InputPort<int>("target_floor"),      
    BT::InputPort<std::string>("config_file"),

    // outputs
    BT::OutputPort<std::string>("door_yaml"),
  };
}

bool SelectFloorConfig::is_absolute(const std::string& p)
{
  return !p.empty() && p.front() == '/';
}

std::string SelectFloorConfig::resolve_path_relative_to_config(const std::string& config_file,
                                                               const std::string& maybe_rel)
{
  if (maybe_rel.empty()) return maybe_rel;
  if (is_absolute(maybe_rel)) return maybe_rel;

  fs::path cfg(config_file);
  fs::path base_dir = cfg.parent_path();  
  fs::path out = base_dir / fs::path(maybe_rel);
  return fs::weakly_canonical(out).string();
}

BT::NodeStatus SelectFloorConfig::tick()
{
  int target_floor;
  if (!getInput("target_floor", target_floor)) {
    return BT::NodeStatus::FAILURE;
  }

  // 0/1/2만 허용 (요청한 규칙)
  if (target_floor < 0 || target_floor > 2) {
    return BT::NodeStatus::FAILURE;
  }

  std::string config_file;
  if (!getInput("config_file", config_file) || config_file.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  fs::path cfg_path(config_file);
  if (!fs::exists(cfg_path) || !fs::is_regular_file(cfg_path)) {
    return BT::NodeStatus::FAILURE;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(config_file);
  } catch (...) {
    return BT::NodeStatus::FAILURE;
  }

  if (!root["floors"] || !root["floors"].IsMap()) {
    return BT::NodeStatus::FAILURE;
  }

  YAML::Node floors = root["floors"];

  YAML::Node matched;
  for (auto it = floors.begin(); it != floors.end(); ++it) {
    YAML::Node v = it->second;
    if (!v || !v["floor_index"]) {
      continue;
    }
    try {
      int idx = v["floor_index"].as<int>();
      if (idx == target_floor) {
        matched = v;
        break;
      }
    } catch (...) {
      continue;
    }
  }

  if (!matched) {
    return BT::NodeStatus::FAILURE;
  }

  if (!matched["door_yaml"]) {
    return BT::NodeStatus::FAILURE;
  }

  std::string door_raw;
  try {
    door_raw = matched["door_yaml"].as<std::string>();
  } catch (...) {
    return BT::NodeStatus::FAILURE;
  }

  std::string door_path = resolve_path_relative_to_config(config_file, door_raw);

  if (!fs::exists(door_path) || !fs::is_regular_file(door_path)) {
    return BT::NodeStatus::FAILURE;
  }

  setOutput("door_yaml", door_path);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace floor_change_bt

#include "elevator_bt/bt_nodes/call_elevator_to_floor.hpp"
#include <cmath>

namespace elevator_bt {

CallElevatorToFloor::CallElevatorToFloor(const std::string& name,
                                         const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config),
  node_(std::make_shared<rclcpp::Node>("call_elevator_to_floor")),
  client_(node_)
{
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);

}

bool CallElevatorToFloor::loadConfig(const std::string& yaml_path)
{
  try {
    auto y = YAML::LoadFile(yaml_path);
    auto e = y["elevator"];
    cfg_.ns     = e["namespace"].as<std::string>();
    cfg_.floors = e["floor_heights_m"].as<std::vector<double>>();
    if (e["cabin_z_tolerance"]) cfg_.tol    = e["cabin_z_tolerance"].as<double>();
    if (e["settle_time_sec"])   cfg_.settle = e["settle_time_sec"].as<double>();
    return true;
  } catch(const std::exception& ex) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to read %s : %s", yaml_path.c_str(), ex.what());
    return false;
  }
}

BT::NodeStatus CallElevatorToFloor::onStart()
{
  std::string yaml_path;
  if (!getInput<std::string>("elevator_yaml", yaml_path)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing port: elevator_yaml");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput<int>("floor_idx", target_floor_)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing port: floor_idx");
    return BT::NodeStatus::FAILURE;
  }
  getInput<double>("timeout", timeout_); 

  if (!loadConfig(yaml_path)) {
    return BT::NodeStatus::FAILURE;
  }

  std::string ns_override;
  if (getInput<std::string>("elevator_ns", ns_override) && !ns_override.empty()) {
    cfg_.ns = ns_override;
  }

  if (target_floor_ < 0 || target_floor_ >= static_cast<int>(cfg_.floors.size())) {
    RCLCPP_ERROR(node_->get_logger(), "floor_idx %d out of range", target_floor_);
    return BT::NodeStatus::FAILURE;
  }

  client_.setNamespace(cfg_.ns);

  target_z_ = cfg_.floors[target_floor_];

  client_.cmdFloor(target_floor_);
  client_.cmdZ(target_z_);

  exec_->spin_some();

  t0_ = std::chrono::steady_clock::now();
  last_log_ = t0_;
  settle_started_ = false;

  warmup_deadline_ = t0_ + std::chrono::seconds(2);

  RCLCPP_INFO(node_->get_logger(), "Elevator NS: %s, call to floor %d (Z=%.3f)",
              cfg_.ns.c_str(), target_floor_, target_z_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CallElevatorToFloor::onRunning()
{
  exec_->spin_some();

  const auto now = std::chrono::steady_clock::now();

  const double elapsed = std::chrono::duration<double>(now - t0_).count();
  if (elapsed > timeout_) {
    RCLCPP_ERROR(node_->get_logger(), "Elevator timeout after %.1f s", elapsed);
    return BT::NodeStatus::FAILURE;
  }

  const double z = client_.cabinZ();
  if (std::isnan(z)) {
    if (now < warmup_deadline_) {
      return BT::NodeStatus::RUNNING;
    }
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *(node_->get_clock()), 2000,
                         "[CallElevatorToFloor] cabin_z is still NaN");
    return BT::NodeStatus::RUNNING;
  }

  if (std::fabs(z - target_z_) <= cfg_.tol) {
    if (!settle_started_) {
      settle_started_ = true;
      settle_t_ = now;
    }
    const double settled = std::chrono::duration<double>(now - settle_t_).count();
    if (settled >= cfg_.settle) {
      RCLCPP_INFO(node_->get_logger(), "Elevator reached floor %d (z=%.3f)", target_floor_, z);
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    settle_started_ = false;
  }

  if (std::chrono::duration<double>(now - last_log_).count() > 1.0) {
    RCLCPP_INFO(node_->get_logger(),
      "[CallElevatorToFloor] z=%.3f, target=%.3f, diff=%.3f, tol=%.3f, inside=%s",
      z, target_z_, std::fabs(z - target_z_), cfg_.tol,
      (std::fabs(z - target_z_) <= cfg_.tol ? "yes":"no"));
    last_log_ = now;
  }

  return BT::NodeStatus::RUNNING;
}

void CallElevatorToFloor::onHalted()
{
}

} // namespace elevator_bt

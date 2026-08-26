#include "elevator_bt/bt_nodes/wait_door_open.hpp"
#include <cmath>

namespace elevator_bt {

WaitDoorOpen::WaitDoorOpen(const std::string& name,
                           const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config),
  node_(std::make_shared<rclcpp::Node>("wait_door_open")),
  client_(node_) {}

bool WaitDoorOpen::loadYaml(const std::string& path, std::string& ns_out)
{
  try {
    auto y = YAML::LoadFile(path);
    auto e = y["elevator"];
    ns_out = e["namespace"].as<std::string>();
    if (e["door_settle_sec"]) settle_sec_ = e["door_settle_sec"].as<double>();
    return true;
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(node_->get_logger(), "[WaitDoorOpen] YAML read fail: %s", ex.what());
    return false;
  }
}

BT::NodeStatus WaitDoorOpen::onStart()
{
  std::string yaml_path, ns_override;
  if (!getInput<std::string>("elevator_yaml", yaml_path)) {
    RCLCPP_ERROR(node_->get_logger(), "[WaitDoorOpen] Missing port: elevator_yaml");
    return BT::NodeStatus::FAILURE;
  }
  getInput<std::string>("elevator_ns", ns_override);

  if (!getInput<bool>("want_open", want_open_)) {
    RCLCPP_ERROR(node_->get_logger(), "[WaitDoorOpen] Missing port: want_open");
    return BT::NodeStatus::FAILURE;
  }

  getInput<double>("open_threshold", threshold_);

  getInput<double>("timeout", timeout_);

  std::string ns_cfg;
  if (!loadYaml(yaml_path, ns_cfg)) return BT::NodeStatus::FAILURE;
  if (!ns_override.empty()) ns_cfg = ns_override;
  client_.setNamespace(ns_cfg);

  t0_ = std::chrono::steady_clock::now();
  settle_started_ = false;
  last_log_ = t0_;

  RCLCPP_INFO(node_->get_logger(),
              "[WaitDoorOpen] ns=%s want_open=%s thr=%.3f timeout=%.1f settle=%.2f",
              ns_cfg.c_str(), want_open_ ? "true" : "false",
              threshold_, timeout_, settle_sec_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitDoorOpen::onRunning()
{
  rclcpp::spin_some(node_);
  const auto now = std::chrono::steady_clock::now();
  const double elapsed = std::chrono::duration<double>(now - t0_).count();
  if (elapsed > timeout_) {
    RCLCPP_ERROR(node_->get_logger(), "[WaitDoorOpen] timeout after %.1f s", elapsed);
    return BT::NodeStatus::FAILURE;
  }

  const double extent = client_.doorOpenExtent();
  
  const bool condition_met = want_open_ ? (extent >= threshold_)
                                        : (extent <= threshold_);

  if (condition_met) {
    if (!settle_started_) {
      settle_started_ = true;
      settle_t_ = now;
    }
    const double hold = std::chrono::duration<double>(now - settle_t_).count();
    if (hold >= settle_sec_) {
      RCLCPP_INFO(node_->get_logger(),
                  "[WaitDoorOpen] condition met. extent=%.3f thr=%.3f want_open=%d",
                  extent, threshold_, static_cast<int>(want_open_));
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    settle_started_ = false;
  }

  if (std::chrono::duration<double>(now - last_log_).count() > 1.0) {
    RCLCPP_INFO(node_->get_logger(),
                "[WaitDoorOpen] extent=%.3f thr=%.3f want_open=%d inside=%s",
                extent, threshold_, static_cast<int>(want_open_),
                condition_met ? "yes":"no");
    last_log_ = now;
  }
  return BT::NodeStatus::RUNNING;
}
void WaitDoorOpen::onHalted()
{
}

} // namespace elevator_bt

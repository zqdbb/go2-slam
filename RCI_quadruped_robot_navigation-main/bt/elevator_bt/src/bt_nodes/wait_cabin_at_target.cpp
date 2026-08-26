#include "elevator_bt/bt_nodes/wait_cabin_at_target.hpp"

namespace elevator_bt {

WaitCabinAtTarget::WaitCabinAtTarget(const std::string& name,
                                     const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config),
  node_(std::make_shared<rclcpp::Node>("wait_cabin_at_target")),
  exec_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()),
  client_(node_)   
  
{
  exec_->add_node(node_);
}

bool WaitCabinAtTarget::loadYaml(const std::string& path, std::string& ns_out)
{
  try{
    auto y = YAML::LoadFile(path);
    auto e = y["elevator"];
    ns_out   = e["namespace"].as<std::string>();
    floors_  = e["floor_heights_m"].as<std::vector<double>>();
    if(e["cabin_z_tolerance"]) tol_    = e["cabin_z_tolerance"].as<double>();
    if(e["settle_time_sec"])   settle_ = e["settle_time_sec"].as<double>();
    return true;
  }catch(const std::exception& ex){
    RCLCPP_ERROR(node_->get_logger(), "[WaitCabinAtTarget] YAML read fail: %s", ex.what());
    return false;
  }
}

BT::NodeStatus WaitCabinAtTarget::onStart()
{
  std::string yaml_path, ns_override;
  if(!getInput<std::string>("elevator_yaml", yaml_path)){
    RCLCPP_ERROR(node_->get_logger(), "[WaitCabinAtTarget] Missing port: elevator_yaml");
    return BT::NodeStatus::FAILURE;
  }
  if(!getInput<int>("floor_idx", target_)){
    RCLCPP_ERROR(node_->get_logger(), "[WaitCabinAtTarget] Missing port: floor_idx");
    return BT::NodeStatus::FAILURE;
  }
  getInput<std::string>("elevator_ns", ns_override);
  getInput<double>("timeout", timeout_);

  std::string ns_cfg;
  if(!loadYaml(yaml_path, ns_cfg)) {
    return BT::NodeStatus::FAILURE;
  }
  if(!ns_override.empty()) ns_cfg = ns_override;
  client_.setNamespace(ns_cfg);

  if(target_ < 0 || target_ >= static_cast<int>(floors_.size())){
    RCLCPP_ERROR(node_->get_logger(), "[WaitCabinAtTarget] floor_idx out of range");
    return BT::NodeStatus::FAILURE;
  }

  exec_->spin_some();

  t0_ = std::chrono::steady_clock::now();
  last_log_ = t0_;
  warmup_deadline_ = t0_ + std::chrono::seconds(2);
  settle_started_ = false;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitCabinAtTarget::onRunning()
{
  exec_->spin_some();

  auto now = std::chrono::steady_clock::now();
  double elapsed = std::chrono::duration<double>(now - t0_).count();
  if(elapsed > timeout_){
    RCLCPP_ERROR(node_->get_logger(), "[WaitCabinAtTarget] timeout");
    return BT::NodeStatus::FAILURE;
  }

  double z = client_.cabinZ();
  if(std::isnan(z)) {
    if (now < warmup_deadline_) {
      return BT::NodeStatus::RUNNING; 
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *(node_->get_clock()), 2000,
                           "[WaitCabinAtTarget] cabin_z is NaN");
      return BT::NodeStatus::RUNNING;
    }
  }

  double target_z = floors_[target_];
  if(std::fabs(z - target_z) <= tol_){
    if(!settle_started_){
      settle_started_ = true;
      settle_t_ = now;
    }
    double hold = std::chrono::duration<double>(now - settle_t_).count();
    if(hold >= settle_){
      RCLCPP_INFO(node_->get_logger(), "[WaitCabinAtTarget] reached z=%.3f", z);
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    settle_started_ = false;
  }

  if (std::chrono::duration<double>(now - last_log_).count() > 1.0) {
    RCLCPP_INFO(node_->get_logger(),
      "[WaitCabinAtTarget] z=%.3f, target=%.3f, diff=%.3f, tol=%.3f, inside=%s",
      z, target_z, std::fabs(z - target_z), tol_,
      (std::fabs(z - target_z) <= tol_ ? "yes":"no"));
    last_log_ = now;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitCabinAtTarget::onHalted()
{
}

} // namespace elevator_bt

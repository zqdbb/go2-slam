#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <mutex>
#include <vector>
#include <string>
#include <cmath>

#include "elevator_bt/bt_nodes/elevator_client.hpp"

namespace elevator_bt {

class WaitCabinAtTarget : public BT::StatefulActionNode {
public:
  WaitCabinAtTarget(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("elevator_yaml"),
      BT::InputPort<std::string>("elevator_ns"),
      BT::InputPort<int>("floor_idx"),
      BT::InputPort<double>("timeout")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;

  ElevatorClient client_;

  std::vector<double> floors_;
  double tol_{0.03};
  double settle_{0.8};
  int    target_{0};
  double timeout_{20.0};

  std::chrono::steady_clock::time_point t0_;
  std::chrono::steady_clock::time_point settle_t_;
  std::chrono::steady_clock::time_point last_log_;
  std::chrono::steady_clock::time_point warmup_deadline_;
  bool settle_started_{false};

  bool loadYaml(const std::string& path, std::string& ns_out);
};

} // namespace elevator_bt

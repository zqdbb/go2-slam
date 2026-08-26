#pragma once
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>  // ★ 추가
#include <yaml-cpp/yaml.h>
#include "elevator_bt/bt_nodes/elevator_client.hpp"

namespace elevator_bt {

struct ElevatorConfig {
  std::string ns;
  std::vector<double> floors;
  double tol = 0.03;
  double settle = 0.8;
};

class CallElevatorToFloor : public BT::StatefulActionNode {
public:
  CallElevatorToFloor(const std::string& name, const BT::NodeConfiguration& config);

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
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_; // ★ 추가
  ElevatorClient client_;
  ElevatorConfig cfg_;

  int target_floor_{0};
  double target_z_{0.0};
  double timeout_{20.0};

  // ★ 실시간(steady_clock) 유지
  std::chrono::steady_clock::time_point t0_;
  std::chrono::steady_clock::time_point settle_t_;
  std::chrono::steady_clock::time_point last_log_;
  bool settle_started_{false};

  // (선택) 첫 샘플 웜업 마감시간 (steady_clock)
  std::chrono::steady_clock::time_point warmup_deadline_;

  bool loadConfig(const std::string& yaml_path);
};

} // namespace elevator_bt

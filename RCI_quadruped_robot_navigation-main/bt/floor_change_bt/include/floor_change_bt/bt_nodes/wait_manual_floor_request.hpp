#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <mutex>

namespace floor_change_bt
{

class WaitManualFloorRequest : public BT::StatefulActionNode
{
public:
  WaitManualFloorRequest(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("wait_manual_floor_request_bt");

    std::string elev_topic, stairs_topic;
    if (!getInput("elevator_topic", elev_topic) || !getInput("stairs_topic", stairs_topic)) {
      throw BT::RuntimeError("Missing required input ports: elevator_topic / stairs_topic");
    }

    elev_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      elev_topic, rclcpp::QoS(10),
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (chosen_) return;
        chosen_ = true;
        mode_ = "elevator";
        floor_ = msg->data;
      });

    stairs_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
      stairs_topic, rclcpp::QoS(10),
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (chosen_) return;
        chosen_ = true;
        mode_ = "stairs";
        floor_ = msg->data;
      });
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("elevator_topic"),
      BT::InputPort<std::string>("stairs_topic"),
      BT::OutputPort<std::string>("mode"),
      BT::OutputPort<int>("target_floor")
    };
  }

  BT::NodeStatus onStart() override
  {
    std::lock_guard<std::mutex> lk(mtx_);
    chosen_ = false;
    mode_.clear();
    floor_ = 0;
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    rclcpp::spin_some(node_);

    std::lock_guard<std::mutex> lk(mtx_);
    if (!chosen_) return BT::NodeStatus::RUNNING;

    RCLCPP_INFO(node_->get_logger(), "manual select mode=[%s], target_floor=%d",
                mode_.c_str(), floor_);
                
    setOutput("mode", mode_);
    setOutput("target_floor", floor_);
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override {}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr elev_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr stairs_sub_;

  std::mutex mtx_;
  bool chosen_{false};
  std::string mode_;
  int floor_{0};
};

} // namespace floor_change_bt

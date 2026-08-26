#include "elevator_bt/bt_nodes/wait_floor_command.hpp"

namespace elevator_bt {

using std_msgs::msg::Int32;

WaitFloorCommand::WaitFloorCommand(const std::string& name,
                                   const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config)
{
  node_ = std::make_shared<rclcpp::Node>("wait_floor_command");
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);
}

void WaitFloorCommand::ensureSubscription()
{
  if (sub_) {
    return;
  }

  if (!getInput("topic_name", topic_name_) || topic_name_.empty()) {
    topic_name_ = "/elevator/floor_request";  
  }

  sub_ = node_->create_subscription<Int32>(
    topic_name_, rclcpp::QoS(10),
    [this](const Int32::SharedPtr msg)
    {
      floor_.store(msg->data, std::memory_order_relaxed);
      got_msg_.store(true, std::memory_order_relaxed);
      RCLCPP_INFO(node_->get_logger(),
        "[WaitFloorCommand] got floor request: %d", msg->data);
    });
}

BT::NodeStatus WaitFloorCommand::onStart()
{
  got_msg_.store(false, std::memory_order_relaxed);
  ensureSubscription();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitFloorCommand::onRunning()
{
  exec_->spin_some();

  if (!got_msg_.load(std::memory_order_relaxed)) {
    return BT::NodeStatus::RUNNING;
  }

  int floor = floor_.load(std::memory_order_relaxed);
  setOutput("target_floor", floor);

  RCLCPP_INFO(node_->get_logger(),
    "[WaitFloorCommand] set target_floor = %d", floor);

  return BT::NodeStatus::SUCCESS;
}

void WaitFloorCommand::onHalted()
{
  got_msg_.store(false, std::memory_order_relaxed);
}

} // namespace elevator_bt

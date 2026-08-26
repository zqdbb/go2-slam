#include "stair_bt/bt_nodes/get_target_floor_from_topic.hpp"

namespace stair_bt
{

GetTargetFloorFromTopic::GetTargetFloorFromTopic(
    const std::string & name,
    const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  node_ = std::make_shared<rclcpp::Node>("get_target_floor_from_topic");
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);
}

void GetTargetFloorFromTopic::floorCallback(
    const std_msgs::msg::Int32::SharedPtr msg)
{
  last_floor_ = msg->data;
  have_msg_ = true;
  RCLCPP_INFO(node_->get_logger(),
              "[GetTargetFloorFromTopic] received target_floor=%d", last_floor_);
}

BT::NodeStatus GetTargetFloorFromTopic::onStart()
{
  have_msg_ = false;

  getInput<std::string>("topic_name", topic_name_);

  double timeout = 0.0;
  if (getInput<double>("timeout", timeout) && timeout > 0.0) {
    timeout_sec_ = timeout;
  } else {
    timeout_sec_ = 0.0;  
  }

  sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    topic_name_, 10,
    std::bind(&GetTargetFloorFromTopic::floorCallback, this, std::placeholders::_1));

  start_time_ = node_->now();

  RCLCPP_INFO(node_->get_logger(),
              "[GetTargetFloorFromTopic] waiting on %s (timeout=%.1f)",
              topic_name_.c_str(), timeout_sec_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetTargetFloorFromTopic::onRunning()
{
  exec_->spin_some();

  if (have_msg_) {
    setOutput<int>("target_floor", last_floor_);
    RCLCPP_INFO(node_->get_logger(),
                "[GetTargetFloorFromTopic] set target_floor=%d", last_floor_);
    return BT::NodeStatus::SUCCESS;
  }

  if (timeout_sec_ > 0.0) {
    double elapsed = (node_->now() - start_time_).seconds();
    if (elapsed > timeout_sec_) {
      RCLCPP_WARN(node_->get_logger(),
                  "[GetTargetFloorFromTopic] timeout %.1f s", elapsed);
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void GetTargetFloorFromTopic::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "[GetTargetFloorFromTopic] halted");
  sub_.reset();
  have_msg_ = false;
}

}  // namespace stair_bt

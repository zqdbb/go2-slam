#include "elevator_bt/bt_nodes/get_current_floor.hpp"

#include <tf2/exceptions.h>

namespace elevator_bt
{

GetCurrentFloor::GetCurrentFloor(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  node_(rclcpp::Node::make_shared("elevator_get_current_floor"))
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList GetCurrentFloor::providedPorts()
{
  return {
    BT::InputPort<std::string>("map_frame", "map"),
    BT::InputPort<std::string>("base_frame", "base_link"),
    BT::InputPort<double>("floor0_z", 0.0, "floor 0 height"),
    BT::InputPort<double>("floor1_z", 5.0, "floor 1 height"),
    BT::InputPort<double>("floor2_z", 10.0, "floor 2 height"),
    BT::OutputPort<int>("current_floor")};
}

BT::NodeStatus GetCurrentFloor::onStart()
{
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetCurrentFloor::onRunning()
{
  rclcpp::spin_some(node_);
  std::string map_frame = "map", base_frame = "base_link";
  double f0 = 0.0, f1 = 5.0, f2 = 10.0;
  getInput("map_frame", map_frame);
  getInput("base_frame", base_frame);
  getInput("floor0_z", f0);
  getInput("floor1_z", f1);
  getInput("floor2_z", f2);
  (void)f0;

  try {
    const auto tf = tf_buffer_->lookupTransform(map_frame, base_frame, tf2::TimePointZero);
    const double z = tf.transform.translation.z;
    const int floor = z < f1 ? 0 : (z < f2 ? 1 : 2);
    setOutput("current_floor", floor);
    RCLCPP_INFO(node_->get_logger(), "Robot z=%.3f, current floor=%d", z, floor);
    return BT::NodeStatus::SUCCESS;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "Waiting for TF %s -> %s: %s", map_frame.c_str(), base_frame.c_str(), ex.what());
    return BT::NodeStatus::RUNNING;
  }
}

void GetCurrentFloor::onHalted() {}

}  // namespace elevator_bt

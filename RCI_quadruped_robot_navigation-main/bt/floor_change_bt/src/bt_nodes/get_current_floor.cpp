#include "floor_change_bt/bt_nodes/get_current_floor.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <sstream>
#include <cstdint>

namespace floor_change_bt
{

static std::string make_unique_node_name()
{

  std::ostringstream oss;
  oss << "get_current_floor_" << std::hex
      << static_cast<std::uintptr_t>(reinterpret_cast<std::uintptr_t>(&oss));
  return oss.str();
}

GetCurrentFloor::GetCurrentFloor(const std::string& name,
                                 const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config)
{
  node_ = std::make_shared<rclcpp::Node>(make_unique_node_name());
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList GetCurrentFloor::providedPorts()
{
  return {
    BT::InputPort<std::string>("map_frame", "map"),
    BT::InputPort<std::string>("base_frame", "base_link"),
    BT::InputPort<double>("floor0_z", 0.0, ""),
    BT::InputPort<double>("floor1_z", 5.0, ""),
    BT::InputPort<double>("floor2_z", 10.0, ""),
    BT::OutputPort<int>("current_floor")
  };
}

BT::NodeStatus GetCurrentFloor::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "[GetCurrentFloor] onStart");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetCurrentFloor::onRunning()
{
  std::string map_frame = "map";
  std::string base_frame = "base_link";
  getInput("map_frame", map_frame);
  getInput("base_frame", base_frame);

  double f0 = 0.0, f1 = 5.0, f2 = 10.0;
  getInput("floor0_z", f0);
  getInput("floor1_z", f1);
  getInput("floor2_z", f2);

  geometry_msgs::msg::TransformStamped tf;

  try {
    tf = tf_buffer_->lookupTransform(map_frame, base_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(),
      1000,
      "[GetCurrentFloor] TF %s->%s 아직 없음: %s",
      map_frame.c_str(), base_frame.c_str(), ex.what());
    return BT::NodeStatus::RUNNING;  
  }

  const double z = tf.transform.translation.z;

  int floor = 0;
  if (z < f1) {
    floor = 0;
  } else if (z < f2) {
    floor = 1;
  } else {
    floor = 2;
  }

  setOutput("current_floor", floor);

  RCLCPP_INFO(node_->get_logger(),
              "[GetCurrentFloor] z=%.3f -> floor=%d", z, floor);

  return BT::NodeStatus::SUCCESS;
}

void GetCurrentFloor::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "[GetCurrentFloor] halted");
}

}  // namespace floor_change_bt
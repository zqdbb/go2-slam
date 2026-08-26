#include "elevator_bt/bt_nodes/wait_robot_inside_cabin.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

namespace elevator_bt {

WaitRobotInsideCabin::WaitRobotInsideCabin(const std::string& name,
                                           const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config)
{
  node_ = std::make_shared<rclcpp::Node>("wait_robot_inside_cabin");
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true);
}

BT::NodeStatus WaitRobotInsideCabin::onStart()
{
  std::string yaml_path;
  if (!getInput("elevator_yaml", yaml_path)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[WaitRobotInsideCabin] missing elevator_yaml");
    return BT::NodeStatus::FAILURE;
  }

  getInput("world_frame", world_frame_);
  getInput("base_frame",  base_frame_);
  getInput("timeout", timeout_);

  if (!load_zones_config(yaml_path, zones_)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[WaitRobotInsideCabin] failed to load zones from %s",
                 yaml_path.c_str());
    return BT::NodeStatus::FAILURE;
  }

  t0_ = std::chrono::steady_clock::now();
  last_log_ = t0_;

  RCLCPP_INFO(node_->get_logger(),
              "[WaitRobotInsideCabin] world_frame=%s, base_frame=%s, "
              "cabin[%.2f,%.2f]x[%.2f,%.2f]",
              world_frame_.c_str(), base_frame_.c_str(),
              zones_.cabin_min_x, zones_.cabin_max_x,
              zones_.cabin_min_y, zones_.cabin_max_y);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitRobotInsideCabin::onRunning()
{
  exec_->spin_some(std::chrono::milliseconds(0));
  auto now = std::chrono::steady_clock::now();

  geometry_msgs::msg::TransformStamped tf;
  try
  {
    tf = tf_buffer_->lookupTransform(
        world_frame_, base_frame_, tf2::TimePointZero);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_DEBUG(node_->get_logger(),
                 "[WaitRobotInsideCabin] TF lookup failed: %s", ex.what());
    return BT::NodeStatus::RUNNING;
  }

  const double x = tf.transform.translation.x;
  const double y = tf.transform.translation.y;

  const bool inside =
      (x >= zones_.cabin_min_x && x <= zones_.cabin_max_x &&
       y >= zones_.cabin_min_y && y <= zones_.cabin_max_y);

  if (inside) {
    RCLCPP_INFO(node_->get_logger(),
                "[WaitRobotInsideCabin] robot inside cabin (x=%.3f,y=%.3f)",
                x, y);
    return BT::NodeStatus::SUCCESS;
  }

  const double elapsed = std::chrono::duration<double>(now - t0_).count();
  if (elapsed > timeout_) {
    RCLCPP_WARN(node_->get_logger(),
                "[WaitRobotInsideCabin] timeout (%.1f s), x=%.3f,y=%.3f",
                elapsed, x, y);
    return BT::NodeStatus::FAILURE;
  }

  if (std::chrono::duration<double>(now - last_log_).count() > 1.0) {
    RCLCPP_INFO(node_->get_logger(),
                "[WaitRobotInsideCabin] waiting, x=%.3f,y=%.3f, "
                "cabin[%.2f,%.2f]x[%.2f,%.2f]",
                x, y,
                zones_.cabin_min_x, zones_.cabin_max_x,
                zones_.cabin_min_y, zones_.cabin_max_y);
    last_log_ = now;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitRobotInsideCabin::onHalted()
{
}

} // namespace elevator_bt

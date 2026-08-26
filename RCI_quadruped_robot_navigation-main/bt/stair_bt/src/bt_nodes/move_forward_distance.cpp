#include "stair_bt/bt_nodes/move_forward_distance.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

namespace stair_bt
{

MoveForwardDistance::MoveForwardDistance(const std::string & name,
                                         const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  node_ = std::make_shared<rclcpp::Node>("move_forward_distance");
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);

  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus MoveForwardDistance::onStart()
{
  if (!getInput<double>("distance", target_distance_) ||
      target_distance_ <= 0.0)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "[MoveForwardDistance] distance 입력이 없거나 <= 0");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput<double>("linear_speed", linear_speed_)) {
    linear_speed_ = 0.2;
  } else {
    linear_speed_ = std::abs(linear_speed_);
  }

  getInput<std::string>("odom_frame", odom_frame_);
  getInput<std::string>("base_frame", base_frame_);

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      odom_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[MoveForwardDistance] TF %s->%s 실패: %s",
                 odom_frame_.c_str(), base_frame_.c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }

  start_x_ = tf.transform.translation.x;
  start_y_ = tf.transform.translation.y;

  start_time_ = node_->now();
  started_ = true;

  RCLCPP_INFO(node_->get_logger(),
              "[MoveForwardDistance] 시작 dist=%.2f m", target_distance_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveForwardDistance::onRunning()
{
  exec_->spin_some();

  if (!started_) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[MoveForwardDistance] onRunning before onStart");
    return BT::NodeStatus::FAILURE;
  }

  double elapsed = (node_->now() - start_time_).seconds();
  if (elapsed > timeout_sec_) {
    RCLCPP_WARN(node_->get_logger(),
                "[MoveForwardDistance] timeout %.1f s", elapsed);
    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      odom_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[MoveForwardDistance] TF %s->%s 실패: %s",
                 odom_frame_.c_str(), base_frame_.c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }

  double x = tf.transform.translation.x;
  double y = tf.transform.translation.y;

  double dx = x - start_x_;
  double dy = y - start_y_;
  double dist = std::sqrt(dx * dx + dy * dy);

  if (dist >= target_distance_) {
    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);
    RCLCPP_INFO(node_->get_logger(),
                "[MoveForwardDistance] 완료 dist=%.3f m", dist);
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear_speed_;
  cmd_pub_->publish(cmd);

  return BT::NodeStatus::RUNNING;
}

void MoveForwardDistance::onHalted()
{
  geometry_msgs::msg::Twist stop;
  cmd_pub_->publish(stop);
  started_ = false;
  RCLCPP_INFO(node_->get_logger(), "[MoveForwardDistance] halted");
}

}  // namespace stair_bt

#include "stair_bt/bt_nodes/turn_relative.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

namespace stair_bt
{

static double yaw_from_quat(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
  return yaw;
}

static double normalize_angle(double a)
{
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

TurnRelative::TurnRelative(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config)
{
  node_ = std::make_shared<rclcpp::Node>("turn_relative");
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);

  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus TurnRelative::onStart()
{
  exec_->spin_some();

  double angle_deg = 0.0;
  if (!getInput<double>("angle_deg", angle_deg))
  {
    RCLCPP_ERROR(node_->get_logger(), "[TurnRelative] angle_deg 입력 필요");
    return BT::NodeStatus::FAILURE;
  }

  getInput<double>("angular_speed", angular_speed_);
  getInput<double>("yaw_tolerance", yaw_tolerance_);

  double t = 0.0;
  if (getInput<double>("timeout", t) && t > 0.0) timeout_sec_ = t;
  else timeout_sec_ = 0.0;

  getInput<std::string>("ref_frame", ref_frame_);
  getInput<std::string>("base_frame", base_frame_);

  angular_speed_ = std::abs(angular_speed_);
  target_delta_  = angle_deg * M_PI / 180.0;  // deg -> rad

  start_yaw_ready_ = false;
  start_time_ = node_->now();

  RCLCPP_INFO(node_->get_logger(),
              "[TurnRelative] 시작: angle=%.1f deg (%.3f rad), ref=%s base=%s",
              angle_deg, target_delta_, ref_frame_.c_str(), base_frame_.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TurnRelative::onRunning()
{
  exec_->spin_some();

  // timeout
  if (timeout_sec_ > 0.0) {
    double elapsed = (node_->now() - start_time_).seconds();
    if (elapsed > timeout_sec_) {
      geometry_msgs::msg::Twist stop;
      cmd_pub_->publish(stop);
      RCLCPP_WARN(node_->get_logger(), "[TurnRelative] timeout %.1f s", elapsed);
      return BT::NodeStatus::FAILURE;
    }
  }

  // TF: ref_frame -> base_frame (없으면 기다림)
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(ref_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "[TurnRelative] TF %s->%s 대기중: %s",
      ref_frame_.c_str(), base_frame_.c_str(), ex.what());
    return BT::NodeStatus::RUNNING;
  }

  double yaw_now = yaw_from_quat(tf.transform.rotation);

  if (!start_yaw_ready_) {
    start_yaw_ = yaw_now;
    start_yaw_ready_ = true;
    return BT::NodeStatus::RUNNING;
  }

  double turned = normalize_angle(yaw_now - start_yaw_);
  double remain = normalize_angle(target_delta_ - turned);

  if (std::fabs(remain) < yaw_tolerance_) {
    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);
    RCLCPP_INFO(node_->get_logger(),
                "[TurnRelative] 완료: turned=%.3f target=%.3f remain=%.3f",
                turned, target_delta_, remain);
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = (remain > 0.0) ? angular_speed_ : -angular_speed_;
  cmd_pub_->publish(cmd);

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "[TurnRelative] 진행중: turned=%.3f target=%.3f remain=%.3f cmd_z=%.3f",
    turned, target_delta_, remain, cmd.angular.z);

  return BT::NodeStatus::RUNNING;
}

void TurnRelative::onHalted()
{
  geometry_msgs::msg::Twist stop;
  cmd_pub_->publish(stop);
  start_yaw_ready_ = false;
  RCLCPP_INFO(node_->get_logger(), "[TurnRelative] halted");
}

}  // namespace stair_bt

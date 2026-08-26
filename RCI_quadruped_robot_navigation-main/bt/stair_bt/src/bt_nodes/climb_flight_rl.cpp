#include "stair_bt/bt_nodes/climb_flight_rl.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <limits>

namespace stair_bt
{

ClimbFlightRL::ClimbFlightRL(const std::string & name,
                             const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  node_ = std::make_shared<rclcpp::Node>("climb_flight_rl");
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);

  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  node_->declare_parameter<double>("delta_z_mid", 2.5);
  node_->declare_parameter<double>("delta_z_top", 5.0);
  node_->declare_parameter<double>("z_tolerance", 0.02);

  world_frame_   = "map";
  base_frame_    = "base_link";
  timeout_sec_   = 0.0;
  linear_speed_  = 0.5;
  direction_sign_ = +1;

  target_level_.clear();
  target_dz_     = 0.0;
  z_tolerance_   = 0.02;

  reference_z_   = 0.0;
  have_reference_z_ = false;

  started_ = false;
}

BT::NodeStatus ClimbFlightRL::onStart()
{
  getInput<std::string>("world_frame", world_frame_);
  getInput<std::string>("base_frame",  base_frame_);

  // timeout
  double timeout = 0.0;
  if (getInput<double>("timeout", timeout) && timeout > 0.0) timeout_sec_ = timeout;
  else timeout_sec_ = 0.0;

  // direction
  std::string direction = "up";
  getInput<std::string>("direction", direction);
  direction_sign_ = (direction == "down") ? -1 : +1;

  double sp_default = 0.5;
  if (getInput<double>("linear_speed", sp_default)) sp_default = std::abs(sp_default);
  else sp_default = 0.5;

  double sp_up = sp_default;
  double sp_down = sp_default;
  auto res_up   = getInput<double>("linear_speed_up", sp_up);
  auto res_down = getInput<double>("linear_speed_down", sp_down);

  const bool has_up   = res_up.has_value();
  const bool has_down = res_down.has_value();

  if (direction_sign_ > 0 && has_up)        linear_speed_ = std::abs(sp_up);
  else if (direction_sign_ < 0 && has_down) linear_speed_ = std::abs(sp_down);
  else                                      linear_speed_ = sp_default;

  // target_level
  if (!getInput<std::string>("target_level", target_level_)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[ClimbFlightRL] target_level 입력 필요 (mid/top)");
    return BT::NodeStatus::FAILURE;
  }

  z_tolerance_ = node_->get_parameter("z_tolerance").as_double();


  double dz_from_entry = std::numeric_limits<double>::quiet_NaN();
  {
    double in = 0.0;
    auto r = getInput<double>("dz_from_entry", in);
    if (r.has_value() && std::isfinite(in)) {
      dz_from_entry = in;
    }
  }

  if (!std::isfinite(dz_from_entry)) {
    if (target_level_ == "mid") dz_from_entry = node_->get_parameter("delta_z_mid").as_double();
    else if (target_level_ == "top") dz_from_entry = node_->get_parameter("delta_z_top").as_double();
    else {
      RCLCPP_ERROR(node_->get_logger(),
                   "[ClimbFlightRL] target_level='%s' (mid/top만 허용)",
                   target_level_.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  target_dz_ = dz_from_entry * direction_sign_;

  have_reference_z_ = false;

  if (target_level_ == "top") {
    double ref_in = 0.0;
    if (getInput<double>("reference_z", ref_in) && std::isfinite(ref_in)) {
      reference_z_ = ref_in;
      have_reference_z_ = true;
    }
  }

  started_ = false;

  RCLCPP_INFO(node_->get_logger(),
              "[ClimbFlightRL] onStart: level=%s dir=%s speed=%.2f dz_from_entry=%.2f target_dz=%.2f tol=%.3f",
              target_level_.c_str(), direction.c_str(),
              linear_speed_, dz_from_entry, target_dz_, z_tolerance_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ClimbFlightRL::onRunning()
{
  exec_->spin_some();

  // TF
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(world_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "[ClimbFlightRL] TF %s->%s 대기중: %s",
      world_frame_.c_str(), base_frame_.c_str(), ex.what());
    return BT::NodeStatus::RUNNING;
  }

  // start time
  if (!started_) {
    start_time_ = node_->now();
    started_ = true;
  }

  if (!have_reference_z_) {
    if (target_level_ == "mid") {
      reference_z_ = tf.transform.translation.z;
      setOutput<double>("reference_z", reference_z_);
      have_reference_z_ = true;

      RCLCPP_INFO(node_->get_logger(),
                  "[ClimbFlightRL] (mid) reference_z 캡처: %.3f", reference_z_);
    } else {
      reference_z_ = tf.transform.translation.z;
      have_reference_z_ = true;
      RCLCPP_WARN(node_->get_logger(),
                  "[ClimbFlightRL] (top) reference_z 입력 없음 -> fallback 캡처. "
                  "계단 시작 기준이 아닐 수 있음(확실하지 않음). ref=%.3f", reference_z_);
    }
  }

  // timeout
  if (timeout_sec_ > 0.0) {
    double elapsed = (node_->now() - start_time_).seconds();
    if (elapsed > timeout_sec_) {
      RCLCPP_WARN(node_->get_logger(), "[ClimbFlightRL] timeout %.1f s", elapsed);
      geometry_msgs::msg::Twist stop;
      cmd_pub_->publish(stop);
      return BT::NodeStatus::FAILURE;
    }
  }

  const double z  = tf.transform.translation.z;
  const double dz = z - reference_z_;

  bool arrived = false;
  if (direction_sign_ > 0) {
    arrived = (dz >= (target_dz_ - z_tolerance_));
  } else {
    arrived = (dz <= (target_dz_ + z_tolerance_));
  }

  if (arrived) {
    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);
    RCLCPP_INFO(node_->get_logger(),
                "[ClimbFlightRL] 완료: z=%.3f dz=%.3f target_dz=%.3f tol=%.3f (ref=%.3f)",
                z, dz, target_dz_, z_tolerance_, reference_z_);
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear_speed_;
  cmd_pub_->publish(cmd);

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "[ClimbFlightRL] 진행중: z=%.3f dz=%.3f target=%.3f ref=%.3f speed=%.2f",
    z, dz, target_dz_, reference_z_, linear_speed_);

  return BT::NodeStatus::RUNNING;
}

void ClimbFlightRL::onHalted()
{
  geometry_msgs::msg::Twist stop;
  cmd_pub_->publish(stop);

  started_ = false;
  have_reference_z_ = false;
  RCLCPP_INFO(node_->get_logger(), "[ClimbFlightRL] halted");
}

}  // namespace stair_bt

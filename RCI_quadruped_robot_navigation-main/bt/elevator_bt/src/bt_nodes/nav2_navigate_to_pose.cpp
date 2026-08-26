#include "elevator_bt/bt_nodes/nav2_navigate_to_pose.hpp"

#include <chrono>
#include <cmath>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace elevator_bt
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;

Nav2NavigateToPose1::Nav2NavigateToPose1(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  logger_(rclcpp::get_logger("Nav2NavigateToPose1")),
  goal_sent_(false),
  timeout_(rclcpp::Duration::from_seconds(300.0))  
{
  node_ = rclcpp::Node::make_shared("nav2_navigate_to_pose_bt_node");

  action_client_ = rclcpp_action::create_client<NavigateToPose>(
    node_,
    "navigate_to_pose");  

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  node_->declare_parameter("success_radius", 0.4);  
  success_radius_ = node_->get_parameter("success_radius").as_double();
}

BT::NodeStatus Nav2NavigateToPose1::onStart()
{
  if (!getInput("goal", goal_pose_))
  {
    RCLCPP_ERROR(logger_, "[Nav2NavigateToPose1] missing input [goal]");
    return BT::NodeStatus::FAILURE;
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(logger_, "[Nav2NavigateToPose1] navigate_to_pose action server not available");
    return BT::NodeStatus::FAILURE;
  }

  NavigateToPose::Goal nav_goal;
  nav_goal.pose = goal_pose_;

  auto send_goal_options =
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  auto future_goal_handle =
    action_client_->async_send_goal(nav_goal, send_goal_options);

  auto ret = rclcpp::spin_until_future_complete(
    node_, future_goal_handle, std::chrono::seconds(2));

  if (ret != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      logger_,
      "[Nav2NavigateToPose1] Failed to send goal (ret=%d)", static_cast<int>(ret));
    return BT::NodeStatus::FAILURE;
  }

  current_goal_handle_ = future_goal_handle.get();
  if (!current_goal_handle_)
  {
    RCLCPP_ERROR(logger_, "[Nav2NavigateToPose1] Goal was rejected by server");
    return BT::NodeStatus::FAILURE;
  }

  goal_sent_       = true;
  goal_start_time_ = node_->now();

  RCLCPP_INFO(logger_, "[Nav2NavigateToPose1] Goal sent");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Nav2NavigateToPose1::onRunning()
{
  if (!goal_sent_ || !current_goal_handle_)
  {
    RCLCPP_ERROR(logger_, "[Nav2NavigateToPose1] onRunning called without a goal");
    return BT::NodeStatus::FAILURE;
  }

  if (node_->now() - goal_start_time_ > std::chrono::seconds(120))
  {
    RCLCPP_WARN(logger_, "[Nav2NavigateToPose1] navigation timeout, canceling goal");
    action_client_->async_cancel_goal(current_goal_handle_);
    goal_sent_ = false;
    current_goal_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::TransformStamped tf;
  try
  {
    const std::string frame = goal_pose_.header.frame_id;
    tf = tf_buffer_->lookupTransform(
      frame, "base_link", tf2::TimePointZero);
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 2000,
      "[Nav2NavigateToPose1] waiting TF (%s->base_link): %s",
      goal_pose_.header.frame_id.c_str(), ex.what());

    return BT::NodeStatus::RUNNING;
  }

  const double rx = tf.transform.translation.x;
  const double ry = tf.transform.translation.y;

  const double gx = goal_pose_.pose.position.x;
  const double gy = goal_pose_.pose.position.y;

  const double dist = std::hypot(gx - rx, gy - ry);

  if (dist < success_radius_)
  {
    RCLCPP_INFO(
      logger_,
      "[Nav2NavigateToPose1] close enough to goal (dist=%.3f < %.3f), treat as SUCCESS",
      dist, success_radius_);

    if (current_goal_handle_)
    {
      action_client_->async_cancel_goal(current_goal_handle_);
    }
    goal_sent_ = false;
    current_goal_handle_.reset();
    return BT::NodeStatus::SUCCESS;
  }

  auto result_future =
    action_client_->async_get_result(current_goal_handle_);

  auto ret = rclcpp::spin_until_future_complete(
    node_, result_future, std::chrono::milliseconds(10));

  if (ret == rclcpp::FutureReturnCode::TIMEOUT)
  {
    return BT::NodeStatus::RUNNING;
  }

  if (ret != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(
      logger_,
      "[Nav2NavigateToPose1] Failed to get result (ret=%d)", static_cast<int>(ret));
    goal_sent_ = false;
    current_goal_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }

  auto wrapped_result = result_future.get();
  auto code = wrapped_result.code;

  RCLCPP_INFO(
    logger_,
    "[Nav2NavigateToPose1] navigation finished with code %d",
    static_cast<int>(code));

  if (code == rclcpp_action::ResultCode::CANCELED)
  {
    RCLCPP_WARN(logger_, "[Nav2NavigateToPose1] navigation CANCELED");
    goal_sent_ = false;
    current_goal_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }

  goal_sent_ = false;
  current_goal_handle_.reset();
  return BT::NodeStatus::SUCCESS;
}

void Nav2NavigateToPose1::onHalted()
{
  if (goal_sent_ && current_goal_handle_)
  {
    RCLCPP_INFO(logger_, "[Nav2NavigateToPose1] Halted, cancel goal");
    action_client_->async_cancel_goal(current_goal_handle_);
  }
  goal_sent_ = false;
  current_goal_handle_.reset();
}

}  // namespace elevator_bt

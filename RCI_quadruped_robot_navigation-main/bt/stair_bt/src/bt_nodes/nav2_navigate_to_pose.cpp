#include "stair_bt/bt_nodes/nav2_navigate_to_pose.hpp"

#include <chrono>
#include <cmath>

namespace stair_bt
{

Nav2NavigateToPose2::Nav2NavigateToPose2(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config),
  logger_(rclcpp::get_logger("Nav2NavigateToPose2"))
{
  node_ = rclcpp::Node::make_shared("nav2_navigate_to_pose_bt_node");

  action_client_ = rclcpp_action::create_client<NavigateToPose>(
    node_, "navigate_to_pose");

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus Nav2NavigateToPose2::onStart()
{
  if (!getInput("entry_pose", goal_pose_))
  {
    RCLCPP_ERROR(logger_, "[Nav2NavigateToPose2] missing input [entry_pose]");
    return BT::NodeStatus::FAILURE;
  }

  getInput<std::string>("base_frame", base_frame_);
  getInput<double>("success_radius", success_radius_);
  getInput<double>("timeout", timeout_sec_);

  if (success_radius_ < 0.0) success_radius_ = std::abs(success_radius_);
  if (timeout_sec_ <= 0.0) timeout_sec_ = 120.0;

  if (goal_pose_.header.frame_id.empty())
  {
    RCLCPP_ERROR(logger_, "[Nav2NavigateToPose2] entry_pose.header.frame_id is empty");
    return BT::NodeStatus::FAILURE;
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(logger_, "[Nav2NavigateToPose2] navigate_to_pose action server not available");
    return BT::NodeStatus::FAILURE;
  }


  goal_sent_ = false;
  current_goal_handle_.reset();
  goal_start_time_ = node_->now();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Nav2NavigateToPose2::onRunning()
{
  if ((node_->now() - goal_start_time_).seconds() > timeout_sec_)
  {
    RCLCPP_WARN(logger_, "[Nav2NavigateToPose2] timeout %.1f s", timeout_sec_);
    if (goal_sent_ && current_goal_handle_)
    {
      action_client_->async_cancel_goal(current_goal_handle_);
    }
    goal_sent_ = false;
    current_goal_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::TransformStamped tf;
  try
  {
    const std::string frame = goal_pose_.header.frame_id;
    tf = tf_buffer_->lookupTransform(frame, base_frame_, tf2::TimePointZero);
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 2000,
      "[Nav2NavigateToPose2] waiting TF (%s->%s): %s",
      goal_pose_.header.frame_id.c_str(), base_frame_.c_str(), ex.what());
    return BT::NodeStatus::RUNNING;
  }

  const double rx = tf.transform.translation.x;
  const double ry = tf.transform.translation.y;
  const double rz = tf.transform.translation.z;

  if (!goal_sent_)
  {

    goal_pose_.pose.position.z = rz;

    NavigateToPose::Goal nav_goal;
    nav_goal.pose = goal_pose_;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    auto future_goal_handle = action_client_->async_send_goal(nav_goal, send_goal_options);

    auto ret = rclcpp::spin_until_future_complete(
      node_, future_goal_handle, std::chrono::seconds(2));

    if (ret != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(
        logger_,
        "[Nav2NavigateToPose2] Failed to send goal (ret=%d)", static_cast<int>(ret));
      return BT::NodeStatus::FAILURE;
    }

    current_goal_handle_ = future_goal_handle.get();
    if (!current_goal_handle_)
    {
      RCLCPP_ERROR(logger_, "[Nav2NavigateToPose2] Goal was rejected by server");
      return BT::NodeStatus::FAILURE;
    }

    goal_sent_ = true;

    RCLCPP_INFO(
      logger_,
      "[Nav2NavigateToPose2] Goal sent (frame=%s, x=%.3f, y=%.3f, z(current)=%.3f), "
      "success_radius=%.2f timeout=%.1f base_frame=%s",
      goal_pose_.header.frame_id.c_str(),
      goal_pose_.pose.position.x,
      goal_pose_.pose.position.y,
      goal_pose_.pose.position.z,
      success_radius_, timeout_sec_, base_frame_.c_str());

    return BT::NodeStatus::RUNNING;
  }

  const double gx = goal_pose_.pose.position.x;
  const double gy = goal_pose_.pose.position.y;

  const double dist = std::hypot(gx - rx, gy - ry);

  if (dist < success_radius_)
  {
    RCLCPP_INFO(
      logger_,
      "[Nav2NavigateToPose2] close enough (dist=%.3f < %.3f), cancel goal and SUCCESS",
      dist, success_radius_);

    action_client_->async_cancel_goal(current_goal_handle_);
    goal_sent_ = false;
    current_goal_handle_.reset();
    return BT::NodeStatus::SUCCESS;
  }

  auto result_future = action_client_->async_get_result(current_goal_handle_);
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
      "[Nav2NavigateToPose2] Failed to get result (ret=%d)", static_cast<int>(ret));
    goal_sent_ = false;
    current_goal_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }

  auto wrapped_result = result_future.get();
  auto code = wrapped_result.code;

  RCLCPP_INFO(
    logger_,
    "[Nav2NavigateToPose2] navigation finished with code %d",
    static_cast<int>(code));

  if (code == rclcpp_action::ResultCode::CANCELED)
  {
    RCLCPP_WARN(logger_, "[Nav2NavigateToPose2] navigation CANCELED");
    goal_sent_ = false;
    current_goal_handle_.reset();
    return BT::NodeStatus::FAILURE;
  }

  goal_sent_ = false;
  current_goal_handle_.reset();
  return BT::NodeStatus::SUCCESS;
}

void Nav2NavigateToPose2::onHalted()
{
  if (goal_sent_ && current_goal_handle_)
  {
    RCLCPP_INFO(logger_, "[Nav2NavigateToPose2] Halted, cancel goal");
    action_client_->async_cancel_goal(current_goal_handle_);
  }
  goal_sent_ = false;
  current_goal_handle_.reset();
}

}  // namespace stair_bt

#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <memory>
#include <string>

namespace stair_bt
{

class Nav2NavigateToPose2 : public BT::StatefulActionNode
{
public:
  Nav2NavigateToPose2(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "entry_pose",
        "Goal pose to navigate (PoseStamped). x,y used; z is ignored."),

      BT::InputPort<std::string>(
        "base_frame", "base_link",
        "Robot base frame for TF lookup"),

      BT::InputPort<double>(
        "success_radius", 0.4,
        "If distance to goal < success_radius, treat as SUCCESS (meters)"),

      // 선택: 전체 타임아웃(초)
      BT::InputPort<double>(
        "timeout", 120.0,
        "Navigation timeout in seconds")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  rclcpp::Logger logger_;
  rclcpp::Node::SharedPtr node_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr current_goal_handle_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::msg::PoseStamped goal_pose_;

  bool goal_sent_{false};
  rclcpp::Time goal_start_time_;

  std::string base_frame_{"base_link"};
  double success_radius_{0.4};
  double timeout_sec_{120.0};
};

}  // namespace stair_bt

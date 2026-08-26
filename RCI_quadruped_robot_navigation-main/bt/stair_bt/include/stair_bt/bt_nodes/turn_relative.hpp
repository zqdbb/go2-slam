#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/twist.hpp>

namespace stair_bt
{

class TurnRelative : public BT::StatefulActionNode
{
public:
  TurnRelative(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("angle_deg", "상대 회전 각도(deg) +:반시계, -:시계"),
      BT::InputPort<double>("angular_speed", 0.5, "회전 속도(rad/s)"),
      BT::InputPort<double>("yaw_tolerance", 0.05, "종료 허용 오차(rad)"),
      BT::InputPort<double>("timeout", 0.0, "타임아웃(sec), 0이면 무제한"),

      BT::InputPort<std::string>("ref_frame", "기준 frame (예: world/map)"),
      BT::InputPort<std::string>("base_frame", "로봇 base frame (예: base_link)")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string ref_frame_{"world"};
  std::string base_frame_{"base_link"};

  double target_delta_{0.0};    
  double angular_speed_{0.5};   
  double yaw_tolerance_{0.05};  
  double timeout_sec_{0.0};     

  double start_yaw_{0.0};
  bool start_yaw_ready_{false};
  rclcpp::Time start_time_;
};

}  // namespace stair_bt

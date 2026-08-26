#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>
#include <memory>

namespace floor_change_bt
{

class PublishInitialPoseFromTF : public BT::StatefulActionNode
{
public:
  PublishInitialPoseFromTF(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // runtime
  int publish_times_{3};
  int published_{0};
  int publish_period_ms_{100};
  rclcpp::Time next_pub_time_;

  std::string topic_{"/initialpose"};
  std::string map_frame_{"map"};
  std::string base_frame_{"base_link"};

  double cov_xy_{0.25};     // m^2
  double cov_yaw_{0.0685};  // rad^2  (대략 (15deg)^2)

  rclcpp::Node::SharedPtr getNodeFromBlackboardOrCreate();
  bool publishOnce();
};

}  // namespace floor_change_bt

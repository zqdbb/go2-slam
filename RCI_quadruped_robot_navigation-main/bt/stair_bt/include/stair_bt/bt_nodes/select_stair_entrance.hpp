#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace stair_bt
{

class SelectStairEntrance : public BT::SyncActionNode
{
public:
  SelectStairEntrance(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("current_floor"),
      BT::InputPort<int>("target_floor"),
      BT::InputPort<std::string>("direction", ""),   
      BT::InputPort<std::string>("stair_yaml", ""),
      BT::InputPort<std::string>("map_frame", "map"),

      BT::OutputPort<std::string>("stair_id"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("entry_pose"),

      BT::OutputPort<double>("entry_yaw"),
      BT::OutputPort<double>("landing_turn1_deg"),
      BT::OutputPort<double>("landing_move_d"),
      BT::OutputPort<double>("landing_turn2_deg"),

      BT::OutputPort<double>("dz_mid"),
      BT::OutputPort<double>("dz_top"),
      BT::OutputPort<double>("flight_speed"),
      BT::OutputPort<double>("landing_speed"),
    };
  }

  BT::NodeStatus tick() override;

private:
  void fillPose(geometry_msgs::msg::PoseStamped& pose,
                const std::string& frame,
                double x, double y, double yaw);

  rclcpp::Node::SharedPtr node_;
};

}  // namespace stair_bt

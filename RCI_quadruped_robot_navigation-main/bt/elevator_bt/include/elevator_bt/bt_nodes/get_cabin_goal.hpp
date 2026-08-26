#pragma once

#include <string>

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace elevator_bt {

class GetCabinGoal : public BT::SyncActionNode  
{
public:
  GetCabinGoal(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("elevator_yaml", "Elevator yaml file path"),
      BT::InputPort<int>("floor_idx", "Current floor index (0,1,2,...)"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
          "cabin_goal", "Goal pose inside the elevator cabin")
    };
  }

  BT::NodeStatus tick() override;

private:
  geometry_msgs::msg::PoseStamped
  makeCabinGoal(const std::string& yaml_path, int floor_idx);
};

}  // namespace elevator_bt

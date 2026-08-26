#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/blackboard.h>   
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

#include "elevator_bt/bt_nodes/wait_robot_near_elevator.hpp"
#include "elevator_bt/bt_nodes/wait_robot_inside_cabin.hpp"
#include "elevator_bt/bt_nodes/wait_robot_outside_elevator.hpp"
#include "elevator_bt/bt_nodes/ok_node.hpp"
#include "elevator_bt/bt_nodes/call_elevator_to_floor.hpp"

#include "elevator_bt/bt_nodes/set_elevator_door.hpp"
#include "elevator_bt/bt_nodes/wait_door_open.hpp"
#include "elevator_bt/bt_nodes/wait_cabin_at_target.hpp"
#include "elevator_bt/bt_nodes/select_nearest_elevator.hpp"
#include "elevator_bt/bt_nodes/nav2_navigate_to_pose.hpp"
#include "elevator_bt/bt_nodes/get_cabin_goal.hpp"
#include "elevator_bt/bt_nodes/wait_floor_command.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("elevator_bt_runner");

  std::string tree_path;
  for (int i = 1; i < argc - 1; ++i)
    if (std::string(argv[i]) == "--tree") { tree_path = argv[i + 1]; break; }
  if (tree_path.empty()) {
    const auto share = ament_index_cpp::get_package_share_directory("elevator_bt");
    tree_path = share + "/bt_trees/tests/navigate_with_elevator.xml";
  }

  const auto share = ament_index_cpp::get_package_share_directory("elevator_bt");
  node->declare_parameter<std::string>("elev_yaml", share + "/config/elevator.yaml");
  node->declare_parameter<std::string>("elevator_ns", "/lift1");  
  node->declare_parameter<int>("target_floor", 1);
  node->declare_parameter<int>("start_floor", 0); 
  
  node->declare_parameter<double>("call_timeout", 120.0);
  node->declare_parameter<double>("open_threshold", 0.5);

  std::string elev_yaml = node->get_parameter("elev_yaml").as_string();
  std::string elevator_ns = node->get_parameter("elevator_ns").as_string();
  int target_floor = node->get_parameter("target_floor").as_int();
  int start_floor = node->get_parameter("start_floor").as_int();
  
  double call_timeout = node->get_parameter("call_timeout").as_double();
  double open_threshold = node->get_parameter("open_threshold").as_double();

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<elevator_bt::Ok>("Ok");
  factory.registerNodeType<elevator_bt::CallElevatorToFloor>("CallElevatorToFloor");
  factory.registerNodeType<elevator_bt::SetElevatorDoor>("SetElevatorDoor");
  factory.registerNodeType<elevator_bt::WaitDoorOpen>("WaitDoorOpen");
  factory.registerNodeType<elevator_bt::WaitCabinAtTarget>("WaitCabinAtTarget");
  factory.registerNodeType<elevator_bt::WaitRobotNearElevator>("WaitRobotNearElevator");
  factory.registerNodeType<elevator_bt::WaitRobotInsideCabin>("WaitRobotInsideCabin");
  factory.registerNodeType<elevator_bt::WaitRobotOutsideElevator>("WaitRobotOutsideElevator");  
  factory.registerNodeType<elevator_bt::SelectNearestElevator>("SelectNearestElevator");
  factory.registerNodeType<elevator_bt::Nav2NavigateToPose1>("Nav2NavigateToPose1");
  factory.registerNodeType<elevator_bt::GetCabinGoal>("GetCabinGoal");
  factory.registerNodeType<elevator_bt::WaitFloorCommand>("WaitFloorCommand");
  
  BT::Tree tree = factory.createTreeFromFile(tree_path);

  tree.rootBlackboard()->set("elev_yaml", elev_yaml);
  tree.rootBlackboard()->set("elevator_ns", elevator_ns); 
  tree.rootBlackboard()->set("target_floor", target_floor);
  tree.rootBlackboard()->set("start_floor", start_floor); 
  tree.rootBlackboard()->set("call_timeout", call_timeout);
  tree.rootBlackboard()->set("open_threshold", open_threshold);

  rclcpp::WallRate rate(20.0);
  while (rclcpp::ok())
  {
    auto status = tree.tickRoot();
    rclcpp::spin_some(node);
    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "BT finished with SUCCESS");
      break;
    }
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
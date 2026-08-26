#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "door_bt/bt_nodes/door_plan_controller.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("door_bt_runner");

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<DoorPlanController>("DoorPlanController");

  node->declare_parameter<std::string>("bt_xml_file", "bt_trees/tests/door_bt_RCI.xml");
  std::string tree_file = node->get_parameter("bt_xml_file").as_string();

  if (!tree_file.empty() && tree_file[0] != '/')
  {
    try
    {
      const auto share = ament_index_cpp::get_package_share_directory("door_bt");
      tree_file = share + "/" + tree_file;
    }
    catch (...)
    {
    }
  }

  RCLCPP_INFO(node->get_logger(), "Using BT XML: %s", tree_file.c_str());

  auto tree = factory.createTreeFromFile(tree_file);

  rclcpp::Rate rate(10.0);
  while (rclcpp::ok())
  {
    tree.tickRoot();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

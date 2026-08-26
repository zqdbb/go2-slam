#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("stair_bt_runner");

  node->declare_parameter<std::string>("bt_xml_file", "/home/home/ros2_ws/src/RCI_quadruped_robot_navigation/bt/stair_bt/bt_trees/stairs.xml");
  std::string xml_file;
  node->get_parameter("bt_xml_file", xml_file);

  if (xml_file.empty())
  {
    RCLCPP_ERROR(node->get_logger(),
                 "[stair_bt_runner] bt_xml_file 파라미터가 비어 있습니다.");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(),
              "[stair_bt_runner] XML: %s", xml_file.c_str());

  BT::BehaviorTreeFactory factory;

  factory.registerFromPlugin("libstair_bt_nav2.so");

  BT::Tree tree;
  try
  {
    tree = factory.createTreeFromFile(xml_file);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(node->get_logger(),
                 "[stair_bt_runner] 트리 생성 실패: %s", e.what());
    return 1;
  }

  rclcpp::Rate rate(10.0);  // 10 Hz tick

  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
  {
    status = tree.tickRoot();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(),
              "[stair_bt_runner] 종료 상태: %s",
              BT::toStr(status, true).c_str());

  rclcpp::shutdown();
  return 0;
}

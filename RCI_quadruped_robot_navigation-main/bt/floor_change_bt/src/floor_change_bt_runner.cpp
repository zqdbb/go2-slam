#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ament_index_cpp/get_package_prefix.hpp>

#include "floor_change_bt/bt_nodes/wait_manual_floor_request.hpp"
#include "floor_change_bt/bt_nodes/check_string_equals.hpp"
#include "floor_change_bt/bt_nodes/get_current_floor.hpp"
#include "floor_change_bt/bt_nodes/publish_door_yaml.hpp"
#include "floor_change_bt/bt_nodes/publish_initial_pose_from_tf.hpp"
#include "floor_change_bt/bt_nodes/select_floor_config.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("floor_change_bt_runner");

  node->declare_parameter<std::string>("bt_xml_file", "");
  node->declare_parameter<std::string>("stair_plugin", "libstair_bt_nav2.so");
  node->declare_parameter<std::string>("elevator_plugin", "libelevator_bt_nav2.so");

  std::string xml_file = node->get_parameter("bt_xml_file").as_string();
  if (xml_file.empty()) {
    RCLCPP_ERROR(node->get_logger(), "bt_xml_file 파라미터가 비어 있습니다.");
    return 1;
  }

  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<floor_change_bt::WaitManualFloorRequest>("WaitManualFloorRequest");
  factory.registerNodeType<floor_change_bt::CheckStringEquals>("CheckStringEquals");
  factory.registerNodeType<floor_change_bt::GetCurrentFloor>("GetCurrentFloor");
  factory.registerNodeType<floor_change_bt::PublishDoorYaml>("PublishDoorYaml");
  factory.registerNodeType<floor_change_bt::PublishInitialPoseFromTF>("PublishInitialPoseFromTF");
  factory.registerNodeType<floor_change_bt::SelectFloorConfig>("SelectFloorConfig");

  const auto stair_prefix = ament_index_cpp::get_package_prefix("stair_bt");
  const auto elev_prefix  = ament_index_cpp::get_package_prefix("elevator_bt");

  const std::string stair_so = stair_prefix + "/lib/" + node->get_parameter("stair_plugin").as_string();
  const std::string elev_so  = elev_prefix  + "/lib/" + node->get_parameter("elevator_plugin").as_string();

  RCLCPP_INFO(node->get_logger(), "Load stair plugin: %s", stair_so.c_str());
  RCLCPP_INFO(node->get_logger(), "Load elevator plugin: %s", elev_so.c_str());

  try {
    factory.registerFromPlugin(stair_so);
    factory.registerFromPlugin(elev_so);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "플러그인 로딩 실패: %s", e.what());
    return 1;
  }

  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(xml_file);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "트리 생성 실패: %s", e.what());
    return 1;
  }

  rclcpp::Rate rate(10.0);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "종료 상태: %s", BT::toStr(status, true).c_str());
  rclcpp::shutdown();
  return 0;
}

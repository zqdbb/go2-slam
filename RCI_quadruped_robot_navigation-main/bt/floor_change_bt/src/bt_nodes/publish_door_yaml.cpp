#include "floor_change_bt/bt_nodes/publish_door_yaml.hpp"

#include <behaviortree_cpp_v3/blackboard.h>

namespace floor_change_bt
{

PublishDoorYaml::PublishDoorYaml(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
  node_ = getNodeFromBlackboardOrCreate();
}

BT::PortsList PublishDoorYaml::providedPorts()
{
  return {
    BT::InputPort<std::string>("yaml", "door yaml path to publish"),
    BT::InputPort<std::string>("topic", "/door_controller/config_yaml", "target topic"),
  };
}

rclcpp::Node::SharedPtr PublishDoorYaml::getNodeFromBlackboardOrCreate()
{
  if (config().blackboard)
  {
    try
    {
      return config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    }
    catch (...) {}

    try
    {
      return config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
    }
    catch (...) {}
  }

  return rclcpp::Node::make_shared("publish_door_yaml_bt_node");
}


void PublishDoorYaml::ensurePublisher(const std::string& topic)
{
  if (topic.empty())
  {
    throw std::runtime_error("PublishDoorYaml: topic is empty");
  }

  if (pub_ && topic == current_topic_)
  {
    return;
  }

  current_topic_ = topic;
  pub_ = node_->create_publisher<std_msgs::msg::String>(current_topic_, rclcpp::QoS(10));
}

BT::NodeStatus PublishDoorYaml::tick()
{
  std::string yaml;
  if (!getInput("yaml", yaml) || yaml.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "[PublishDoorYaml] missing input port: yaml");
    return BT::NodeStatus::FAILURE;
  }

  std::string topic = "/door_controller/config_yaml";
  (void)getInput("topic", topic);

  try
  {
    ensurePublisher(topic);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "[PublishDoorYaml] %s", e.what());
    return BT::NodeStatus::FAILURE;
  }

  if (yaml == last_yaml_)
  {
    return BT::NodeStatus::SUCCESS;
  }

  std_msgs::msg::String msg;
  msg.data = yaml;
  pub_->publish(msg);

  last_yaml_ = yaml;

  RCLCPP_INFO(node_->get_logger(), "[PublishDoorYaml] publish: topic=%s yaml=%s",
              current_topic_.c_str(), yaml.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace floor_change_bt

#include "floor_change_bt/bt_nodes/publish_initial_pose_from_tf.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace floor_change_bt
{

PublishInitialPoseFromTF::PublishInitialPoseFromTF(
  const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config)
{
  node_ = getNodeFromBlackboardOrCreate();

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList PublishInitialPoseFromTF::providedPorts()
{
  return {
    BT::InputPort<std::string>("topic", "/initialpose", "Initial pose topic (AMCL)"),
    BT::InputPort<std::string>("map_frame", "map", "Frame to publish pose in"),
    BT::InputPort<std::string>("base_frame", "base_link", "Robot base frame"),

    BT::InputPort<int>("publish_times", 3, "Publish N times (for reliability)"),
    BT::InputPort<int>("publish_period_ms", 100, "Period between publishes (ms)"),

    BT::InputPort<double>("cov_xy", 0.25, "Covariance for x,y (m^2)"),
    BT::InputPort<double>("cov_yaw", 0.0685, "Covariance for yaw (rad^2)")
  };
}

rclcpp::Node::SharedPtr PublishInitialPoseFromTF::getNodeFromBlackboardOrCreate()
{
  if (config().blackboard)
  {
    try { return config().blackboard->get<rclcpp::Node::SharedPtr>("node"); } catch (...) {}
    try { return config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node"); } catch (...) {}
  }
  return rclcpp::Node::make_shared("publish_initialpose_bt_node");
}

BT::NodeStatus PublishInitialPoseFromTF::onStart()
{
  (void)getInput("topic", topic_);
  (void)getInput("map_frame", map_frame_);
  (void)getInput("base_frame", base_frame_);
  (void)getInput("publish_times", publish_times_);
  (void)getInput("publish_period_ms", publish_period_ms_);
  (void)getInput("cov_xy", cov_xy_);
  (void)getInput("cov_yaw", cov_yaw_);

  if (!pub_ || pub_->get_topic_name() != topic_)
  {
    pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      topic_, rclcpp::QoS(10));
  }

  published_ = 0;
  next_pub_time_ = node_->now();

  if (!publishOnce())
  {
    return BT::NodeStatus::RUNNING;
  }

  published_++;
  next_pub_time_ = node_->now() + rclcpp::Duration::from_nanoseconds(
    static_cast<int64_t>(publish_period_ms_) * 1000LL * 1000LL);

  return (published_ >= publish_times_) ? BT::NodeStatus::SUCCESS
                                       : BT::NodeStatus::RUNNING;
}

BT::NodeStatus PublishInitialPoseFromTF::onRunning()
{
  rclcpp::spin_some(node_);

  if (published_ >= publish_times_)
    return BT::NodeStatus::SUCCESS;

  if (node_->now() < next_pub_time_)
    return BT::NodeStatus::RUNNING;

  if (!publishOnce())
  {
    return BT::NodeStatus::RUNNING;
  }

  published_++;
  next_pub_time_ = node_->now() + rclcpp::Duration::from_nanoseconds(
    static_cast<int64_t>(publish_period_ms_) * 1000LL * 1000LL);

  return (published_ >= publish_times_) ? BT::NodeStatus::SUCCESS
                                       : BT::NodeStatus::RUNNING;
}

void PublishInitialPoseFromTF::onHalted()
{
  // nothing
}

bool PublishInitialPoseFromTF::publishOnce()
{
  geometry_msgs::msg::TransformStamped tf;
  try
  {
    tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
  }
  catch (const tf2::TransformException&)
  {
    return false;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = map_frame_;

  msg.pose.pose.position.x = tf.transform.translation.x;
  msg.pose.pose.position.y = tf.transform.translation.y;
  msg.pose.pose.position.z = 0.0;

  msg.pose.pose.orientation = tf.transform.rotation;

  for (auto & v : msg.pose.covariance) v = 0.0;
  msg.pose.covariance[0]  = cov_xy_;   // x
  msg.pose.covariance[7]  = cov_xy_;   // y
  msg.pose.covariance[35] = cov_yaw_;  // yaw

  pub_->publish(msg);
  return true;
}

}  // namespace floor_change_bt

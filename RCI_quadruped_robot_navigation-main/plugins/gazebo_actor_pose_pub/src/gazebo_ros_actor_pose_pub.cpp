// gazebo_ros_actor_pose_pub.cpp
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>

#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace gazebo_plugins
{
class GazeboRosActorPosePub : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model_->GetWorld();

    // ROS node
    ros_node_ = gazebo_ros::Node::Get(sdf);

    topic_ = sdf->HasElement("topic") ? sdf->Get<std::string>("topic") : ("/" + model_->GetName() + "/pose");
    frame_id_ = sdf->HasElement("frame_id") ? sdf->Get<std::string>("frame_id") : "map";
    update_rate_ = sdf->HasElement("update_rate") ? sdf->Get<double>("update_rate") : 20.0;

    pub_ = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_, rclcpp::QoS(10));

    last_pub_sim_time_ = gazebo::common::Time(0);

    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboRosActorPosePub::OnUpdate, this, std::placeholders::_1));
  }

private:
  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    if (update_rate_ > 0.0) {
      const double dt = (info.simTime - last_pub_sim_time_).Double();
      if (dt < (1.0 / update_rate_)) return;
    }
    last_pub_sim_time_ = info.simTime;

    // 현재 월드 pose
    const auto p = model_->WorldPose();  // ignition::math::Pose3d

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = ros_node_->now();
    msg.header.frame_id = frame_id_;

    msg.pose.position.x = p.Pos().X();
    msg.pose.position.y = p.Pos().Y();
    msg.pose.position.z = p.Pos().Z();
    msg.pose.orientation.x = p.Rot().X();
    msg.pose.orientation.y = p.Rot().Y();
    msg.pose.orientation.z = p.Rot().Z();
    msg.pose.orientation.w = p.Rot().W();

    pub_->publish(msg);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  gazebo::event::ConnectionPtr update_conn_;

  std::string topic_;
  std::string frame_id_;
  double update_rate_{20.0};
  gazebo::common::Time last_pub_sim_time_;
};
GZ_REGISTER_MODEL_PLUGIN(GazeboRosActorPosePub)
}  // namespace gazebo_plugins

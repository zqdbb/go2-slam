#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <string>

namespace gazebo
{

class WorldTfPublisher : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model_->GetWorld();
    node_  = gazebo_ros::Node::Get(sdf);

    // SDF 파라미터
    world_frame_ = sdf->HasElement("world_frame") ?
      sdf->Get<std::string>("world_frame") : "world";

    base_frame_ = sdf->HasElement("base_frame") ?
      sdf->Get<std::string>("base_frame") : "base_link";

    // ★ odom_frame은 선택 사항
    if (sdf->HasElement("odom_frame"))
    {
      odom_frame_ = sdf->Get<std::string>("odom_frame");
      publish_odom_ = true;
    }
    else
    {
      publish_odom_ = false;
    }

    tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&WorldTfPublisher::OnUpdate, this));

    RCLCPP_INFO(node_->get_logger(),
      "[WorldTfPublisher] world_frame=%s, odom_frame=%s, base_frame=%s",
      world_frame_.c_str(),
      publish_odom_ ? odom_frame_.c_str() : "(none)",
      base_frame_.c_str());
  }

private:
  void OnUpdate()
  {
    rclcpp::Time now = node_->get_clock()->now();

    // Gazebo에서 world 기준 로봇 위치/자세 읽기
    ignition::math::Pose3d pose = model_->WorldPose();

    // 편의를 위해 먼저 world->base_link 변환 계산
    geometry_msgs::msg::TransformStamped tf_world_base;
    tf_world_base.header.stamp = now;
    tf_world_base.header.frame_id = world_frame_;
    tf_world_base.child_frame_id  = base_frame_;

    tf_world_base.transform.translation.x = pose.Pos().X();
    tf_world_base.transform.translation.y = pose.Pos().Y();
    tf_world_base.transform.translation.z = pose.Pos().Z();

    tf2::Quaternion q;
    q.setRPY(pose.Rot().Roll(), pose.Rot().Pitch(), pose.Rot().Yaw());
    tf_world_base.transform.rotation.x = q.x();
    tf_world_base.transform.rotation.y = q.y();
    tf_world_base.transform.rotation.z = q.z();
    tf_world_base.transform.rotation.w = q.w();

    if (!publish_odom_)
    {
      // 예전처럼 world -> base_link만 바로 브로드캐스트
      tf_broadcaster_->sendTransform(tf_world_base);
    }
    else
    {
      // 1) world -> odom : 항등 변환
      geometry_msgs::msg::TransformStamped tf_world_odom;
      tf_world_odom.header.stamp = now;
      tf_world_odom.header.frame_id = world_frame_;
      tf_world_odom.child_frame_id  = odom_frame_;
      tf_world_odom.transform.translation.x = 0.0;
      tf_world_odom.transform.translation.y = 0.0;
      tf_world_odom.transform.translation.z = 0.0;
      tf_world_odom.transform.rotation.x = 0.0;
      tf_world_odom.transform.rotation.y = 0.0;
      tf_world_odom.transform.rotation.z = 0.0;
      tf_world_odom.transform.rotation.w = 1.0;

      // 2) odom -> base_link : world->base_link와 동일 (world==odom 가정)
      geometry_msgs::msg::TransformStamped tf_odom_base = tf_world_base;
      tf_odom_base.header.frame_id = odom_frame_;

      tf_broadcaster_->sendTransform(tf_world_odom);
      tf_broadcaster_->sendTransform(tf_odom_base);
    }
  }

  // Gazebo
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr update_conn_;

  // ROS2
  gazebo_ros::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // 파라미터
  std::string world_frame_{"world"};
  std::string base_frame_{"base_link"};
  std::string odom_frame_{"odom"};
  bool publish_odom_{false};
};

GZ_REGISTER_MODEL_PLUGIN(WorldTfPublisher)

}  // namespace gazebo

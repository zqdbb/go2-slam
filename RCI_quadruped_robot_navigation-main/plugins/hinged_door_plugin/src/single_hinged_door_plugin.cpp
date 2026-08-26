#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <string>
#include <stdexcept>
#include <cmath>

namespace gazebo
{

class SingleHingedDoorPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model_->GetWorld();
    node_  = gazebo_ros::Node::Get(sdf);  // <ros><namespace> 사용 가능

    // SDF 인자
    joint_name_ = sdf->HasElement("right_joint") ?
      sdf->Get<std::string>("right_joint") : "right_joint";

    open_angle_ = sdf->HasElement("open_angle") ?
      sdf->Get<double>("open_angle") : 1.57;   // [rad]
    kp_ = sdf->HasElement("kp") ? sdf->Get<double>("kp") : 150.0;
    kd_ = sdf->HasElement("kd") ? sdf->Get<double>("kd") : 80.0;

    jc_ = model_->GetJointController();
    auto j = model_->GetJoint(joint_name_);
    if (!j)
      throw std::runtime_error(
        "SingleHingedDoorPlugin: joint not found: " + joint_name_);

    joint_scoped_ = j->GetScopedName();

    // PID 및 초기 타깃(닫힘=0)
    jc_->SetPositionPID(joint_scoped_, common::PID(kp_, 0.0, kd_));
    jc_->SetPositionTarget(joint_scoped_, 0.0);

    using std::placeholders::_1;
    sub_open_ = node_->create_subscription<std_msgs::msg::Bool>(
      "door_open", rclcpp::QoS(10),
      std::bind(&SingleHingedDoorPlugin::OnDoorOpen, this, _1));

    pub_pos_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("door_pos", 10);

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&SingleHingedDoorPlugin::OnUpdate, this));

    RCLCPP_INFO(node_->get_logger(),
      "[%s] SingleHingedDoorPlugin ready. Topic: %s/door_open",
      model_->GetName().c_str(), node_->get_namespace());
  }

private:
  void OnDoorOpen(const std_msgs::msg::Bool::SharedPtr msg)
  {
    // 열기: +open_angle / 닫기: 0
    const double target = msg->data ? open_angle_ : 0.0;
    jc_->SetPositionTarget(joint_scoped_, target);
    is_open_ = msg->data;
  }

  void OnUpdate()
  {
    auto j = model_->GetJoint(joint_name_);
    if (!j) return;

    std_msgs::msg::Float64MultiArray m;
    m.data.resize(1);
    m.data[0] = j->Position(0);  // [rad]
    pub_pos_->publish(m);
  }

  // --- 멤버들 ---
  // Gazebo
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr update_conn_;
  physics::JointControllerPtr jc_;

  // ROS2
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_open_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pos_;

  // Params
  std::string joint_name_{"right_joint"};
  std::string joint_scoped_;
  double open_angle_{1.57};
  double kp_{150.0}, kd_{80.0};
  bool is_open_{false};
};

GZ_REGISTER_MODEL_PLUGIN(SingleHingedDoorPlugin)

} // namespace gazebo

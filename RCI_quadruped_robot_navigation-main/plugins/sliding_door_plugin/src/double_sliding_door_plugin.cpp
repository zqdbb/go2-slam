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

class DoubleSlidingDoorPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model_->GetWorld();
    node_  = gazebo_ros::Node::Get(sdf);  // <ros><namespace> 사용 가능

    // SDF 인자
    right_joint_name_ = sdf->HasElement("right_joint") ?
      sdf->Get<std::string>("right_joint") : "right_joint";
    left_joint_name_  = sdf->HasElement("left_joint")  ?
      sdf->Get<std::string>("left_joint")  : "left_joint";
    open_dist_ = sdf->HasElement("open_distance") ?
      sdf->Get<double>("open_distance") : 0.75;   // [m]
    kp_ = sdf->HasElement("kp") ? sdf->Get<double>("kp") : 150.0;
    kd_ = sdf->HasElement("kd") ? sdf->Get<double>("kd") : 80.0;

    jc_ = model_->GetJointController();
    auto rj = model_->GetJoint(right_joint_name_);
    auto lj = model_->GetJoint(left_joint_name_);
    if (!rj || !lj)
      throw std::runtime_error(
        "DoubleSlidingDoorPlugin: joint not found: " +
        right_joint_name_ + " / " + left_joint_name_);

    right_scoped_ = rj->GetScopedName();
    left_scoped_  = lj->GetScopedName();

    // PID 및 초기 타깃(닫힘=0)
    jc_->SetPositionPID(right_scoped_, common::PID(kp_, 0.0, kd_));
    jc_->SetPositionPID(left_scoped_,  common::PID(kp_, 0.0, kd_));
    jc_->SetPositionTarget(right_scoped_, 0.0);
    jc_->SetPositionTarget(left_scoped_,  0.0);

    using std::placeholders::_1;
    sub_open_ = node_->create_subscription<std_msgs::msg::Bool>(
      "door_open", rclcpp::QoS(10),
      std::bind(&DoubleSlidingDoorPlugin::OnDoorOpen, this, _1));

    pub_pos_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("door_pos", 10);

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&DoubleSlidingDoorPlugin::OnUpdate, this));

    RCLCPP_INFO(node_->get_logger(),
      "[%s] DoubleSlidingDoorPlugin ready. Topic: %s/door_open",
      model_->GetName().c_str(), node_->get_namespace());
  }

private:
  void OnDoorOpen(const std_msgs::msg::Bool::SharedPtr msg)
  {
    // 열기: 오른쪽 +open_dist, 왼쪽 -open_dist / 닫기: 둘 다 0
    const double r = msg->data ?  open_dist_ : 0.0;
    const double l = msg->data ? -open_dist_ : 0.0;
    jc_->SetPositionTarget(right_scoped_, r);
    jc_->SetPositionTarget(left_scoped_,  l);
    is_open_ = msg->data;
  }

  void OnUpdate()
  {
    auto rj = model_->GetJoint(right_joint_name_);
    auto lj = model_->GetJoint(left_joint_name_);
    if (!rj || !lj) return;

    std_msgs::msg::Float64MultiArray m;
    m.data.resize(2);
    m.data[0] = rj->Position(0);
    m.data[1] = lj->Position(0);
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
  std::string right_joint_name_{"right_joint"};
  std::string left_joint_name_{"left_joint"};
  std::string right_scoped_, left_scoped_;
  double open_dist_{0.75};
  double kp_{400.0}, kd_{40.0};
  bool is_open_{false};
};

GZ_REGISTER_MODEL_PLUGIN(DoubleSlidingDoorPlugin)

} // namespace gazebo

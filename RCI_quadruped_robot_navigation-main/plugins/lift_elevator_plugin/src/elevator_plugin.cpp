#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cmath>

namespace gazebo
{

class ElevatorPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model_->GetWorld();

    // ===== ROS 2 Node (namespace는 SDF의 <ros><namespace>나 상위에서 주입) =====
    ros_node_ = gazebo_ros::Node::Get(sdf);
    RCLCPP_INFO(ros_node_->get_logger(), "ElevatorPlugin for model [%s] starting",
                model_->GetName().c_str());

    // ===== Read SDF parameters =====
    cabin_joint_name_ = sdf->HasElement("cabin_joint") ?
      sdf->Get<std::string>("cabin_joint") : "cabin_joint";
    right_joint_name_ = sdf->HasElement("right_joint") ?
      sdf->Get<std::string>("right_joint") : "right_joint";
    left_joint_name_ = sdf->HasElement("left_joint") ?
      sdf->Get<std::string>("left_joint") : "left_joint";

    door_open_dist_ = sdf->HasElement("door_open_distance") ?
      sdf->Get<double>("door_open_distance") : 0.85;

    // floor heights: "0 3 6" 형태
    if (sdf->HasElement("floor_heights"))
    {
      std::stringstream ss(sdf->Get<std::string>("floor_heights"));
      double z;
      while (ss >> z) floor_heights_.push_back(z);
    }

    // PID gains
    kp_ = sdf->HasElement("kp") ? sdf->Get<double>("kp") : 400.0;
    kd_ = sdf->HasElement("kd") ? sdf->Get<double>("kd") : 40.0;

    // Motion constraints (optional)
    v_max_ = sdf->HasElement("vel_limit") ? sdf->Get<double>("vel_limit") : v_max_;
    a_max_ = sdf->HasElement("acc_limit") ? sdf->Get<double>("acc_limit") : a_max_;
    pos_tol_ = sdf->HasElement("pos_tolerance") ? sdf->Get<double>("pos_tolerance") : pos_tol_;
    kv_hold_ = sdf->HasElement("kv_hold") ? sdf->Get<double>("kv_hold") : kv_hold_;
    kd_hold_ = sdf->HasElement("kd_hold") ? sdf->Get<double>("kd_hold") : kd_hold_;
    // Force-hold gains from SDF (optional)
    kp_hold_force_ = sdf->HasElement("kp_hold_force") ? sdf->Get<double>("kp_hold_force") : kp_hold_force_;
    c_hold_force_  = sdf->HasElement("c_hold_force")  ? sdf->Get<double>("c_hold_force")  : c_hold_force_;

    // Arrival/Hold 히스테리시스 설정 (추가)
    enter_tol_ = pos_tol_;
    exit_tol_  = sdf->HasElement("pos_tolerance_hyst")
                   ? sdf->Get<double>("pos_tolerance_hyst")
                   : 2.0 * enter_tol_;    
    // ---- Ramp params (slide only) ----
    if (sdf->HasElement("ramp_slide_joint")) ramp_slide_joint_name_ = sdf->Get<std::string>("ramp_slide_joint");
    if (sdf->HasElement("ramp_extend"))      ramp_extend_           = sdf->Get<double>("ramp_extend");
    if (sdf->HasElement("ramp_retract"))     ramp_retract_          = sdf->Get<double>("ramp_retract");
    if (sdf->HasElement("ramp_kp"))          ramp_kp_               = sdf->Get<double>("ramp_kp");
    if (sdf->HasElement("ramp_kd"))          ramp_kd_               = sdf->Get<double>("ramp_kd");
    if (sdf->HasElement("ramp_auto"))        ramp_auto_             = sdf->Get<bool>("ramp_auto");
    if (sdf->HasElement("spawn_settle_time"))spawn_settle_time_     = sdf->Get<double>("spawn_settle_time");

    // ===== Get joints =====
    jc_ = model_->GetJointController();

    auto cj = model_->GetJoint(cabin_joint_name_);
    auto rj = model_->GetJoint(right_joint_name_);
    auto lj = model_->GetJoint(left_joint_name_);

    if (!cj || !rj || !lj)
    {
      std::ostringstream oss;
      oss << "Missing joints: "
          << cabin_joint_name_ << " / "
          << right_joint_name_ << " / "
          << left_joint_name_;
      RCLCPP_ERROR(ros_node_->get_logger(), "%s", oss.str().c_str());
      throw std::runtime_error(oss.str());
    }

    cabin_scoped_ = cj->GetScopedName();
    right_scoped_ = rj->GetScopedName();
    left_scoped_  = lj->GetScopedName();

    // ---- Ramp slide joint ----
    if (auto sj = model_->GetJoint(ramp_slide_joint_name_)) {
      ramp_slide_scoped_ = sj->GetScopedName();
      jc_->SetPositionPID(ramp_slide_scoped_, common::PID(ramp_kp_, 0.0, ramp_kd_));
      jc_->SetPositionTarget(ramp_slide_scoped_, ramp_retract_);   // 시작은 수납
    } else {
      RCLCPP_WARN(ros_node_->get_logger(), "Ramp slide joint [%s] not found", ramp_slide_joint_name_.c_str());
    }
    start_time_ = world_->SimTime();


    // Read platform mass (optional, 튜닝 참고용)
    if (auto link = model_->GetLink("platform")) {
      if (link->GetInertial()) {
        m_platform_ = link->GetInertial()->Mass();
      }
    }
    // ===== Register PID & initial targets =====
    jc_->SetPositionPID(cabin_scoped_, common::PID(kp_, 0.0, kd_));
    jc_->SetPositionPID(right_scoped_, common::PID(kp_, 0.0, kd_));
    jc_->SetPositionPID(left_scoped_,  common::PID(kp_, 0.0, kd_));

    // Hold current Z at startup to prevent gravity drop / drift
    {
      auto cj2 = model_->GetJoint(cabin_joint_name_);
      target_z_ = cj2->Position(0);
      z_cmd_ = target_z_;
      z_vel_ = 0.0;
      jc_->SetPositionTarget(cabin_scoped_, z_cmd_);
    }
    last_sim_time_ = world_->SimTime();

    // 초기: 닫힌 상태 / 현재 층
    jc_->SetPositionTarget(right_scoped_, 0.0);
    jc_->SetPositionTarget(left_scoped_,  0.0);

    // ===== ROS 2 subscribers =====
    using std::placeholders::_1;

    sub_cmd_z_ = ros_node_->create_subscription<std_msgs::msg::Float64>(
      "cmd_z", 10, std::bind(&ElevatorPlugin::OnCmdZ, this, _1));

    sub_cmd_floor_ = ros_node_->create_subscription<std_msgs::msg::Int32>(
      "cmd_floor", 10, std::bind(&ElevatorPlugin::OnCmdFloor, this, _1));

    sub_door_open_ = ros_node_->create_subscription<std_msgs::msg::Bool>(
      "door_open", 10, std::bind(&ElevatorPlugin::OnDoorOpen, this, _1));

    // ===== ROS 2 publishers (상태) =====
    pub_cabin_z_ = ros_node_->create_publisher<std_msgs::msg::Float64>("cabin_z", 10);
    pub_door_pos_ = ros_node_->create_publisher<std_msgs::msg::Float64MultiArray>("door_pos", 10);

    // ===== World update hook (상태 퍼블리시) =====
    update_conn_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ElevatorPlugin::OnUpdate, this));

    RCLCPP_INFO(ros_node_->get_logger(),
      "ElevatorPlugin loaded. Topics:\n - %s/cmd_z (Float64)\n - %s/cmd_floor (Int32)\n - %s/door_open (Bool)",
      ros_node_->get_namespace(), ros_node_->get_namespace(), ros_node_->get_namespace());
  }

private:
  // Callbacks
  void OnCmdZ(const std_msgs::msg::Float64::SharedPtr msg)
  {
      arrived_ = false;          // (추가) 도착 래치 해제
      target_z_ = msg->data;     // 목표만 갱신
  }

  void OnCmdFloor(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (floor_heights_.empty()) {
      RCLCPP_WARN_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 2000,
                           "floor_heights is empty; cmd_floor ignored");
      return;
    }
    const int f = msg->data;
    if (f < 0 || f >= static_cast<int>(floor_heights_.size())) {
      RCLCPP_WARN(ros_node_->get_logger(), "Invalid floor %d", f);
      return;
    }
    arrived_ = false;          // (추가) 도착 래치 해제

    target_z_ = floor_heights_[f];
  }

  void OnDoorOpen(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const double r = msg->data ?  door_open_dist_ : 0.0;  // right door
    const double l = msg->data ? -door_open_dist_ : 0.0;  // left door
    jc_->SetPositionTarget(right_scoped_, r);
    jc_->SetPositionTarget(left_scoped_,  l);

    if (!ramp_slide_scoped_.empty())
      jc_->SetPositionTarget(ramp_slide_scoped_, msg->data ? ramp_extend_ : ramp_retract_);
  }

  // === OnUpdate() : Arrival/Hold 래치 버전 ===
  void OnUpdate()
  {
    auto cj = model_->GetJoint(cabin_joint_name_);
    auto rj = model_->GetJoint(right_joint_name_);
    auto lj = model_->GetJoint(left_joint_name_);
    if (!cj || !rj || !lj) return;

    const common::Time now = world_->SimTime();
    const double dt = (now - last_sim_time_).Double();
    last_sim_time_ = now;
    if (dt <= 0.0) return;

    const double z_meas = cj->Position(0);
    const double v_meas = cj->GetVelocity(0);

    if (!arrived_)  // ---- 이동 모드: 프로파일 생성 + 위치제어 ----
    {
      // 프로파일 상태 기반 오차
      double err = target_z_ - z_cmd_;
      const double sgn  = (err >= 0.0) ? 1.0 : -1.0;
      const double dist = std::abs(err);

      // 거리-기반 허용속도(정지거리 보장) + v_max 제한
      double v_allow = std::sqrt(std::max(0.0, 2.0 * a_max_ * std::max(dist - enter_tol_, 0.0)));
      double v_des   = sgn * std::min(v_max_, v_allow);

      // 가속도 제한으로 속도 상태 갱신
      const double dv_max = a_max_ * dt;
      double dv = v_des - z_vel_;
      if (dv >  dv_max) dv =  dv_max;
      if (dv < -dv_max) dv = -dv_max;
      z_vel_ += dv;

      // 위치 적분
      z_cmd_ += z_vel_ * dt;

      // 타깃 넘지 않도록 최종 클램프
      double err_after = target_z_ - z_cmd_;
      if ((err > 0.0 && err_after < 0.0) || (err < 0.0 && err_after > 0.0))
        z_cmd_ = target_z_;

      // --- 위치 제어: 이동 중에는 PD ---
      jc_->SetPositionPID(cabin_scoped_, common::PID(kp_, 0.0, kd_));
      jc_->SetPositionTarget(cabin_scoped_, z_cmd_);

      // --- 도착 판정: 오차·속도 모두 작으면 홀드 모드로 스위칭 ---
      const bool near = (std::abs(target_z_ - z_meas) <= enter_tol_) && (std::abs(v_meas) <= v_eps_);
      if (near) {
        arrived_ = true;
        z_cmd_ = z_meas;    // 실측 위치로 스냅 → 바로 그 자리에서 멈춤
        z_vel_ = 0.0;
        jc_->SetPositionPID(cabin_scoped_, common::PID(0,0,0));
        jc_->SetVelocityPID(cabin_scoped_, common::PID(0.0, 0.0, 0.0));
        jc_->SetVelocityTarget(cabin_scoped_, 0.0);
        }
    }
    else           // ---- 홀드 모드: Force hold (viscous + spring outside deadband) ----
    {
      const double e = target_z_ - z_meas;
      double F = 0.0;

      if (std::abs(e) <= enter_tol_) {
        // 데드밴드 내부: 순수 점성 댐핑으로 에너지 제거 (헌팅 억제)
        F = - c_hold_force_ * v_meas;
      } else {
        // 데드밴드 밖: 스프링 + 점성
        F = (kp_hold_force_ * e) - (c_hold_force_ * v_meas);
      }

      cj->SetForce(0, F);

      // 히스테리시스: tolerance를 벗어나면 이동 모드로 복귀
      if (std::abs(e) > exit_tol_) {
        arrived_ = false;
        z_cmd_ = z_meas;   // 현재 위치에서 다시 프로파일 시작
        z_vel_ = 0.0;
      }
    }

    // 상태 publish (문 상태 포함 — 기존 코드 유지)
    std_msgs::msg::Float64 zmsg; zmsg.data = z_meas;
    pub_cabin_z_->publish(zmsg);

    std_msgs::msg::Float64MultiArray dmsg;
    dmsg.data = { rj->Position(0), lj->Position(0) };
    pub_door_pos_->publish(dmsg);
  }

private:
  // Gazebo
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr update_conn_;
  physics::JointControllerPtr jc_;

  // ROS
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_cmd_z_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr   sub_cmd_floor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr     sub_door_open_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr     pub_cabin_z_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_door_pos_;

  // Joint names & scoped names
  std::string cabin_joint_name_{"cabin_joint"};
  std::string right_joint_name_{"right_joint"};
  std::string left_joint_name_{"left_joint"};
  std::string cabin_scoped_, right_scoped_, left_scoped_;

  // Params
  std::vector<double> floor_heights_;
  double door_open_dist_{0.85};
  double kp_{400.0}, kd_{40.0};
  double target_z_{0.0};
  
  // Arrival / Hold latch (추가)
  bool arrived_{false};
  double enter_tol_{0.005};  // [m] 도착 판정 오차 (보통 pos_tolerance와 동일)
  double exit_tol_{0.010};   // [m] 히스테리시스(도착 해제 오차 = enter의 2배 추천)
  double v_eps_{0.05};       // [m/s] 도착 시 속도 임계

  // 홀드(정지 유지)용 속도 제어 게인
  double kv_hold_{200.0};    // velocity PID Kp
  double kd_hold_{5.0};      // velocity PID Kd
  // --- Force-hold parameters ---
  double kp_hold_force_{10000.0};  // [N/m]  (데드밴드 밖에서만 작동)
  double c_hold_force_{7000.0};    // [N·s/m] 점성 댐핑 (≈ 2*sqrt(k*m)) 권장
  double m_platform_{0.0};         // [kg]   플랫폼 질량(옵션)

  // ---- Sliding ramp (1-DOF) ----
  std::string ramp_slide_joint_name_{"ramp_slide_joint"};
  std::string ramp_slide_scoped_;
  double ramp_extend_{0.55}, ramp_retract_{0.0};   // [m]
  double ramp_kp_{200.0}, ramp_kd_{20.0};          // 슬라이드용 PID
  bool   ramp_auto_{true};                         // 도착+문열림 시 자동 전개
  double spawn_settle_time_{0.3};                  // 스폰 직후 N초는 고정
  gazebo::common::Time start_time_;


  // Motion profile parameters & state
  double v_max_{0.6};   // [m/s]   S-curve/trapezoid max velocity (SDF: <vel_limit>)
  double a_max_{1.2};   // [m/s^2] max acceleration (SDF: <acc_limit>)
  double pos_tol_{0.005}; // [m] position tolerance to consider "arrived" (SDF: <pos_tolerance>)
  double z_cmd_{0.0};   // profiled position command
  double z_vel_{0.0};   // profiled velocity state
  common::Time last_sim_time_;
};

// Register plugin
GZ_REGISTER_MODEL_PLUGIN(ElevatorPlugin)

} // namespace gazebo
//
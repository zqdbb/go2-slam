#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include "bspline_opt/uniform_bspline.h"

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController() : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    final_approach_dist_ = declare_parameter<double>("final_approach_dist", 0.35);
    final_approach_speed_ = declare_parameter<double>("final_approach_speed", 0.30);
    final_min_speed_ = declare_parameter<double>("final_min_speed", 0.50);
    final_alignment_walk_speed_ = declare_parameter<double>("final_alignment_walk_speed", 0.50);
    final_yaw_tolerance_ = declare_parameter<double>("final_yaw_tolerance", 0.0873);
    final_min_vyaw_ = declare_parameter<double>("final_min_vyaw", 0.12);

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/goal", 10,
        std::bind(&ClosedLoopController::goalCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    RCLCPP_INFO(get_logger(), "Closed-loop controller ready");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    cmd_vel_pub_->publish(cmd);
  }

  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);
    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    traj_id_ = msg->traj_id;
    exec_time_ = 0.0;
    last_update_time_ = now();
    receive_traj_ = true;
    RCLCPP_INFO(get_logger(), "Received trajectory %lld, duration %.3fs",
                static_cast<long long>(traj_id_), traj_duration_);
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    have_odom_ = true;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    if (!msg || !std::isfinite(msg->pose.position.x) ||
        !std::isfinite(msg->pose.position.y))
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid navigation goal");
      return;
    }

    goal_pos_ << msg->pose.position.x, msg->pose.position.y;
    goal_yaw_ = tf2::getYaw(msg->pose.orientation);
    have_goal_ = std::isfinite(goal_yaw_);
    final_alignment_active_ = false;
    final_alignment_complete_ = false;
    if (have_goal_)
    {
      RCLCPP_INFO(get_logger(), "Final pose target: (%.2f, %.2f), yaw %.1f deg",
                  goal_pos_.x(), goal_pos_.y(), goal_yaw_ * 180.0 / M_PI);
    }
  }

  void cmdCallback()
  {
    if (!receive_traj_ || !have_odom_)
    {
      publishExecutionFrozen(false);
      publishStop();
      return;
    }
    const auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) dt = 0.0;

    const double goal_distance = have_goal_ ? (goal_pos_ - odom_pos_.head<2>()).norm() : 1e9;
    if (final_alignment_complete_)
    {
      publishExecutionFrozen(false);
      publishStop();
      last_update_time_ = current_time;
      return;
    }
    if (have_goal_ && (final_alignment_active_ || goal_distance <= finish_dist_))
    {
      if (!final_alignment_active_)
      {
        final_alignment_active_ = true;
        RCLCPP_INFO(get_logger(), "Position reached; aligning to final goal yaw");
      }

      const double final_yaw_error = normalizeAngle(goal_yaw_ - odom_yaw_);
      if (std::abs(final_yaw_error) <= final_yaw_tolerance_)
      {
        final_alignment_complete_ = true;
        RCLCPP_INFO(get_logger(), "Navigation goal complete; final yaw error %.1f deg",
                    final_yaw_error * 180.0 / M_PI);
        publishExecutionFrozen(false);
        publishStop();
      }
      else
      {
        double yaw_command = std::clamp(kp_yaw_ * final_yaw_error, -max_vyaw_, max_vyaw_);
        if (std::abs(yaw_command) < final_min_vyaw_)
          yaw_command = std::copysign(final_min_vyaw_, yaw_command);
        geometry_msgs::msg::Twist command;
        if (goal_distance > 0.03)
        {
          const Eigen::Vector2d to_goal = goal_pos_ - odom_pos_.head<2>();
          const Eigen::Vector2d vel_world = to_goal / goal_distance * final_alignment_walk_speed_;
          const double c = std::cos(odom_yaw_);
          const double s = std::sin(odom_yaw_);
          command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
          command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
        }
        else
        {
          // V5 was trained mostly with non-zero linear commands. A small
          // walking command activates the turning gait; position feedback
          // above pulls the robot back as soon as it leaves the goal center.
          command.linear.x = final_alignment_walk_speed_;
        }
        command.angular.z = yaw_command;
        publishExecutionFrozen(false);
        cmd_vel_pub_->publish(command);
      }
      last_update_time_ = current_time;
      return;
    }

    if (have_goal_ && goal_distance <= final_approach_dist_)
    {
      const Eigen::Vector2d to_goal = goal_pos_ - odom_pos_.head<2>();
      double approach_speed = std::min(final_approach_speed_, kp_pos_ * goal_distance);
      approach_speed = std::max(approach_speed, final_min_speed_);
      const Eigen::Vector2d vel_world = to_goal / goal_distance * approach_speed;
      const double desired_yaw = std::atan2(to_goal.y(), to_goal.x());
      const double yaw_error = normalizeAngle(desired_yaw - odom_yaw_);
      const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
      if (std::abs(yaw_error) > heading_error_threshold_)
      {
        publishExecutionFrozen(true);
        publishStop(yaw_command);
      }
      else
      {
        const double c = std::cos(odom_yaw_);
        const double s = std::sin(odom_yaw_);
        geometry_msgs::msg::Twist command;
        command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
        command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
        command.angular.z = yaw_command;
        publishExecutionFrozen(false);
        cmd_vel_pub_->publish(command);
      }
      last_update_time_ = current_time;
      return;
    }

    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    const double yaw_error = normalizeAngle(estimateDesiredYaw(t_eval, pos_des) - odom_yaw_);
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    if (std::abs(yaw_error) > heading_error_threshold_)
    {
      publishExecutionFrozen(true);
      publishStop(yaw_command);
      last_update_time_ = current_time;
      return;
    }

    publishExecutionFrozen(false);
    exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    last_update_time_ = current_time;
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    const Eigen::Vector2d vel_world = clampNorm(
        Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
        std::max(max_vx_, max_vy_));
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    command.angular.z = yaw_command;
    if (exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_)
      command = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector2d goal_pos_{Eigen::Vector2d::Zero()};
  double odom_yaw_{0.0};
  double goal_yaw_{0.0};
  double exec_time_{0.0};
  bool have_goal_{false};
  bool final_alignment_active_{false};
  bool final_alignment_complete_{false};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  double time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;
  double final_approach_dist_, final_approach_speed_, final_min_speed_;
  double final_yaw_tolerance_, final_min_vyaw_, final_alignment_walk_speed_;
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}

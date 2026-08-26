#include "stair_bt/bt_nodes/align_with_stairs.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>

namespace stair_bt
{

static double yaw_from_quat(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
  return yaw;
}

static double normalize_angle(double a)
{
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

AlignWithStairs::AlignWithStairs(const std::string & name,
                                 const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  node_ = std::make_shared<rclcpp::Node>("align_with_stairs");
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);

  cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  started_ = false;
  target_yaw_ = 0.0;
}

BT::NodeStatus AlignWithStairs::onStart()
{
  getInput<std::string>("world_frame", world_frame_);
  getInput<std::string>("base_frame",  base_frame_);

  getInput<double>("yaw_tolerance", yaw_tolerance_);
  getInput<double>("angular_speed", angular_speed_);
  getInput<double>("timeout",  timeout_);

  if (angular_speed_ <= 0.0) angular_speed_ = 0.5;
  angular_speed_ = std::abs(angular_speed_);

  std::string stair_yaml, stair_id, direction;
  if (!getInput<std::string>("stair_yaml", stair_yaml)) {
    RCLCPP_ERROR(node_->get_logger(), "[AlignWithStairs] stair_yaml 입력 없음");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput<std::string>("stair_id", stair_id)) {
    RCLCPP_ERROR(node_->get_logger(), "[AlignWithStairs] stair_id 입력 없음");
    return BT::NodeStatus::FAILURE;
  }
  getInput<std::string>("direction", direction);
  if (direction.empty()) direction = "up";

  try
  {
    YAML::Node root = YAML::LoadFile(stair_yaml);
    YAML::Node stairs = root["stairs"];
    if (!stairs || !stairs[stair_id]) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[AlignWithStairs] stairs[%s] 없음 (yaml=%s)",
                   stair_id.c_str(), stair_yaml.c_str());
      return BT::NodeStatus::FAILURE;
    }

    YAML::Node entry = stairs[stair_id]["entry"];
    if (!entry) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[AlignWithStairs] stairs[%s].entry 없음", stair_id.c_str());
      return BT::NodeStatus::FAILURE;
    }

    // 1) new format: entry.up/down
    if (entry["up"] || entry["down"])
    {
      if (entry[direction] && entry[direction]["yaw"])
      {
        target_yaw_ = normalize_angle(entry[direction]["yaw"].as<double>());
      }
      else if (entry["up"] && entry["up"]["yaw"])
      {
        double up_yaw = entry["up"]["yaw"].as<double>();
        target_yaw_ = (direction == "down")
                        ? normalize_angle(up_yaw + M_PI)
                        : normalize_angle(up_yaw);
        RCLCPP_WARN(node_->get_logger(),
                    "[AlignWithStairs] entry.%s.yaw 없음 -> up yaw로부터 생성 (확실하지 않음)",
                    direction.c_str());
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "[AlignWithStairs] entry yaw 정보 부족");
        return BT::NodeStatus::FAILURE;
      }
    }
    // 2) old format: entry.yaw
    else if (entry["yaw"])
    {
      double entry_yaw = entry["yaw"].as<double>();
      target_yaw_ = (direction == "down")
                      ? normalize_angle(entry_yaw + M_PI)
                      : normalize_angle(entry_yaw);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[AlignWithStairs] entry.yaw 없음");
      return BT::NodeStatus::FAILURE;
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "[AlignWithStairs] YAML 로드/파싱 실패: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }

  start_time_ = node_->now();
  started_ = true;

  RCLCPP_INFO(node_->get_logger(),
              "[AlignWithStairs] 시작: world=%s base=%s target_yaw=%.3f",
              world_frame_.c_str(), base_frame_.c_str(), target_yaw_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AlignWithStairs::onRunning()
{
  exec_->spin_some();

  if (!started_) {
    RCLCPP_ERROR(node_->get_logger(), "[AlignWithStairs] onRunning before onStart");
    return BT::NodeStatus::FAILURE;
  }

  if (timeout_ > 0.0) {
    double elapsed = (node_->now() - start_time_).seconds();
    if (elapsed > timeout_) {
      RCLCPP_WARN(node_->get_logger(), "[AlignWithStairs] timeout %.1f s", elapsed);
      geometry_msgs::msg::Twist stop;
      cmd_pub_->publish(stop);
      return BT::NodeStatus::FAILURE;
    }
  }

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(world_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "[AlignWithStairs] TF %s->%s 대기중: %s",
                         world_frame_.c_str(), base_frame_.c_str(), ex.what());
    return BT::NodeStatus::RUNNING;
  }

  const double yaw = yaw_from_quat(tf.transform.rotation);
  const double diff = normalize_angle(target_yaw_ - yaw);

  if (std::fabs(diff) < yaw_tolerance_) {
    geometry_msgs::msg::Twist stop;
    cmd_pub_->publish(stop);
    RCLCPP_INFO(node_->get_logger(),
                "[AlignWithStairs] 정렬 완료 diff=%.3f", diff);
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = (diff > 0.0 ? angular_speed_ : -angular_speed_);
  cmd_pub_->publish(cmd);

  return BT::NodeStatus::RUNNING;
}

void AlignWithStairs::onHalted()
{
  geometry_msgs::msg::Twist stop;
  cmd_pub_->publish(stop);
  started_ = false;
}

}  // namespace stair_bt

#include "elevator_bt/bt_nodes/select_nearest_elevator.hpp"

#include <tf2/time.h>
#include <tf2/LinearMath/Quaternion.h>

#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <mutex>
#include <chrono>

namespace elevator_bt
{

SelectNearestElevator::SelectNearestElevator(
    const std::string & name,
    const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config),
  lift1_cabin_z_(0.0),
  lift2_cabin_z_(0.0),
  have_lift1_z_(false),
  have_lift2_z_(false)
{
  node_ = std::make_shared<rclcpp::Node>("select_nearest_elevator");

  // TF
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true);

  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);

  sub_lift1_z_ = node_->create_subscription<std_msgs::msg::Float64>(
    "/lift1/cabin_z", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lock(z_mutex_);
      lift1_cabin_z_ = msg->data;
      have_lift1_z_  = true;
    });

  sub_lift2_z_ = node_->create_subscription<std_msgs::msg::Float64>(
    "/lift2/cabin_z", 10,
    [this](const std_msgs::msg::Float64::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lock(z_mutex_);
      lift2_cabin_z_ = msg->data;
      have_lift2_z_  = true;
    });
}

BT::PortsList SelectNearestElevator::providedPorts()
{
  return {
    BT::InputPort<std::string>("world_frame", "map"),
    BT::InputPort<std::string>("base_frame", "base_link"),
    BT::InputPort<int>("current_floor"),
    BT::InputPort<std::string>("lift1_yaml"),
    BT::InputPort<std::string>("lift2_yaml"),
    // outputs
    BT::OutputPort<std::string>("elev_yaml"),
    BT::OutputPort<std::string>("elevator_ns"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("elevator_entrance")
  };
}

bool SelectNearestElevator::loadEntrance(const std::string & yaml_path,
                                         int floor_idx,
                                         EntranceInfo & out_info)
{
  out_info = EntranceInfo{};
  out_info.yaml_path = yaml_path;

  try
  {
    auto y = YAML::LoadFile(yaml_path);
    auto e = y["elevator"];
    if (!e)
    {
      RCLCPP_ERROR(node_->get_logger(),
        "[SelectNearestElevator] 'elevator' key not found in %s",
        yaml_path.c_str());
      return false;
    }

    // namespace
    if (!e["namespace"])
    {
      RCLCPP_ERROR(node_->get_logger(),
        "[SelectNearestElevator] 'namespace' key not found in %s",
        yaml_path.c_str());
      return false;
    }
    out_info.ns = e["namespace"].as<std::string>();

    if (!e["floor_heights_m"] || !e["floor_heights_m"].IsSequence())
    {
      RCLCPP_ERROR(node_->get_logger(),
        "[SelectNearestElevator] 'floor_heights_m' must be a sequence in %s",
        yaml_path.c_str());
      return false;
    }
    auto heights = e["floor_heights_m"];
    if (floor_idx < 0 || floor_idx >= static_cast<int>(heights.size()))
    {
      RCLCPP_ERROR(node_->get_logger(),
        "[SelectNearestElevator] floor_idx %d out of range in floor_heights_m (size=%zu) %s",
        floor_idx, heights.size(), yaml_path.c_str());
      return false;
    }
    out_info.z = heights[floor_idx].as<double>();

    if (!e["entrances"])
    {
      RCLCPP_ERROR(node_->get_logger(),
        "[SelectNearestElevator] 'entrances' key not found in %s",
        yaml_path.c_str());
      return false;
    }

    auto ents = e["entrances"];
    if (!ents.IsSequence())
    {
      RCLCPP_ERROR(node_->get_logger(),
        "[SelectNearestElevator] 'entrances' must be a sequence in %s",
        yaml_path.c_str());
      return false;
    }

    if (floor_idx < 0 || floor_idx >= static_cast<int>(ents.size()))
    {
      RCLCPP_ERROR(node_->get_logger(),
        "[SelectNearestElevator] floor_idx %d out of range (size=%zu) in %s",
        floor_idx, ents.size(), yaml_path.c_str());
      return false;
    }

    auto ent = ents[floor_idx];

    if (!ent["x"] || !ent["y"])
    {
      RCLCPP_ERROR(node_->get_logger(),
        "[SelectNearestElevator] entrances[%d] missing x or y in %s",
        floor_idx, yaml_path.c_str());
      return false;
    }

    out_info.x = ent["x"].as<double>();
    out_info.y = ent["y"].as<double>();

    if (y["zones"] && y["zones"]["cabin_min"] && y["zones"]["cabin_max"])
    {
      auto cabin_min = y["zones"]["cabin_min"];
      auto cabin_max = y["zones"]["cabin_max"];

      double cabin_cx = 0.5 * (cabin_min[0].as<double>() + cabin_max[0].as<double>());
      double cabin_cy = 0.5 * (cabin_min[1].as<double>() + cabin_max[1].as<double>());

      double vx = cabin_cx - out_info.x;
      double vy = cabin_cy - out_info.y;
      out_info.yaw = std::atan2(vy, vx);
    }
    else
    {
      out_info.yaw = 0.0;
    }

    out_info.valid = true;

    RCLCPP_DEBUG(node_->get_logger(),
      "[SelectNearestElevator] %s floor=%d -> entrance (%.3f, %.3f, %.3f, yaw=%.3f)",
      yaml_path.c_str(), floor_idx, out_info.x, out_info.y, out_info.z, out_info.yaw);

    return true;
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(node_->get_logger(),
      "[SelectNearestElevator] Failed to read %s : %s",
      yaml_path.c_str(), ex.what());
    return false;
  }
}

BT::NodeStatus SelectNearestElevator::tick()
{
  RCLCPP_INFO(node_->get_logger(), "[SelectNearestElevator] TICK");

  using namespace std::chrono_literals;

  exec_->spin_some(0ms);

  double cabin_z1 = 0.0, cabin_z2 = 0.0;
  bool have1 = false, have2 = false;
  {
    std::lock_guard<std::mutex> lock(z_mutex_);
    cabin_z1 = lift1_cabin_z_;
    cabin_z2 = lift2_cabin_z_;
    have1    = have_lift1_z_;
    have2    = have_lift2_z_;
  }

  if (!have1 || !have2)
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[SelectNearestElevator] waiting cabin_z topics: lift1=%d lift2=%d",
      have1, have2);
    return BT::NodeStatus::RUNNING;
  }

  std::string world_frame = "map";
  std::string base_frame  = "base_link";
  getInput("world_frame", world_frame);
  getInput("base_frame",  base_frame);

  int current_floor = 0;
  if (!getInput("current_floor", current_floor))
  {
    RCLCPP_ERROR(node_->get_logger(),
      "[SelectNearestElevator] Missing input port: current_floor");
    return BT::NodeStatus::FAILURE;
  }

  std::string lift1_yaml, lift2_yaml;
  if (!getInput("lift1_yaml", lift1_yaml) || !getInput("lift2_yaml", lift2_yaml))
  {
    RCLCPP_ERROR(node_->get_logger(),
      "[SelectNearestElevator] Missing input ports: lift1_yaml / lift2_yaml");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::TransformStamped tf;
  try
  {
    tf = tf_buffer_->lookupTransform(world_frame, base_frame, tf2::TimePointZero);
  }
  catch (const tf2::TransformException & ex)
  {
    RCLCPP_WARN(node_->get_logger(),
      "[SelectNearestElevator] TF lookup (%s -> %s) failed: %s",
      world_frame.c_str(), base_frame.c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }

  const double rx = tf.transform.translation.x;
  const double ry = tf.transform.translation.y;
  const double rz = tf.transform.translation.z;

  EntranceInfo e1, e2;
  if (!loadEntrance(lift1_yaml, current_floor, e1) ||
      !loadEntrance(lift2_yaml, current_floor, e2))
  {
    return BT::NodeStatus::FAILURE;
  }

  const double dxy1 = std::hypot(e1.x - rx, e1.y - ry);
  const double dxy2 = std::hypot(e2.x - rx, e2.y - ry);

  const double dz1 = std::fabs(cabin_z1 - rz);
  const double dz2 = std::fabs(cabin_z2 - rz);

  const double dz_eps  = 0.20;  
  const double dxy_eps = 0.05;  

  const EntranceInfo * chosen = nullptr;
  std::string chosen_ns;
  double chosen_cabin_z = 0.0;

  if (dz1 + dz_eps < dz2)
  {
    chosen         = &e1;
    chosen_ns      = e1.ns;
    chosen_cabin_z = cabin_z1;
  }
  else if (dz2 + dz_eps < dz1)
  {
    chosen         = &e2;
    chosen_ns      = e2.ns;
    chosen_cabin_z = cabin_z2;
  }
  else
  {
    if (dxy1 <= dxy2 + dxy_eps)
    {
      chosen         = &e1;
      chosen_ns      = e1.ns;
      chosen_cabin_z = cabin_z1;
    }
    else
    {
      chosen         = &e2;
      chosen_ns      = e2.ns;
      chosen_cabin_z = cabin_z2;
    }
  }

  if (!chosen)
  {
    RCLCPP_ERROR(node_->get_logger(), "[SelectNearestElevator] internal error: chosen is null");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("elev_yaml", chosen->yaml_path);
  setOutput("elevator_ns", chosen_ns);

  geometry_msgs::msg::PoseStamped entrance_pose;
  entrance_pose.header.frame_id = world_frame;
  entrance_pose.header.stamp    = node_->get_clock()->now();
  entrance_pose.pose.position.x = chosen->x;
  entrance_pose.pose.position.y = chosen->y;
  entrance_pose.pose.position.z = chosen_cabin_z;  

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, chosen->yaw);
  entrance_pose.pose.orientation.x = q.x();
  entrance_pose.pose.orientation.y = q.y();
  entrance_pose.pose.orientation.z = q.z();
  entrance_pose.pose.orientation.w = q.w();

  setOutput("elevator_entrance", entrance_pose);

  RCLCPP_INFO(node_->get_logger(),
    "[SelectNearestElevator] Robot(%.3f, %.3f, %.3f), "
    "lift1: dxy=%.2f, dz=%.2f (cabin_z=%.2f), "
    "lift2: dxy=%.2f, dz=%.2f (cabin_z=%.2f) -> choose %s",
    rx, ry, rz,
    dxy1, dz1, cabin_z1,
    dxy2, dz2, cabin_z2,
    chosen_ns.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace elevator_bt

#pragma once

#include <behaviortree_cpp_v3/action_node.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <mutex>
#include <atomic>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>
#include <limits>

class DoorPlanController : public BT::StatefulActionNode
{
public:
  DoorPlanController(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  struct DoorDef
  {
    std::string ns;  
    double p1x{}, p1y{}, p2x{}, p2y{};
    double cx{}, cy{}, radius{};
  };

  struct DoorIO
  {
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_cmd;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_pos;

    bool has_pos{false};
    double pos_abs_max{0.0};
    rclcpp::Time last_pos_time;
  };

  struct DoorRuntime
  {
    bool opening{false};   
    bool opened{false};    
    bool closing{false};   

    rclcpp::Time last_cmd_time;
    bool has_last_cmd{false};
    bool last_cmd_open{false};
  };

private:
  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_plan_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_config_yaml_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enabled_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // shared
  std::mutex mtx_;
  nav_msgs::msg::Path::SharedPtr last_plan_;

  // config 
  std::string plan_topic_{"/plan"};
  std::string world_frame_{"map"};
  std::string base_frame_{"base_link"};

  // runtime control topics
  std::string config_yaml_topic_{"/door_controller/config_yaml"};
  std::string enabled_topic_{"/door_controller/enabled"};

  // enable gate 
  bool enabled_{true};

  // YAML hot-reload
  std::mutex cfg_mtx_;
  std::atomic<bool> reload_requested_{false};
  std::string pending_yaml_;

  bool require_feedback_{true};
  double open_threshold_{0.05};
  double close_threshold_{0.02};
  double feedback_timeout_s_{5.0};
  double cooldown_s_{1.0};

  // YAML path 
  std::string doors_yaml_{""};

  // doors
  std::vector<DoorDef> doors_;

  // IO + runtime
  std::unordered_map<std::string, DoorIO> io_;              // ns -> IO
  std::unordered_map<std::string, DoorRuntime> rt_;         // ns -> runtime


  std::unordered_set<std::string> processed_in_plan_;

  // cooldown
  std::unordered_map<std::string, rclcpp::Time> done_time_; // ns -> last done time

  // plan change detect
  rclcpp::Time last_plan_stamp_;
  size_t last_plan_size_{0};

private:
  // callbacks
  void planCb(const nav_msgs::msg::Path::SharedPtr msg);
  void doorPosCb(const std::string& ns, const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void configYamlCb(const std_msgs::msg::String::SharedPtr msg);
  void enabledCb(const std_msgs::msg::Bool::SharedPtr msg);

  // load/parse
  std::string resolvePathMaybeInShare(const std::string& path) const;
  bool loadDoorsFromYaml(const std::string& yaml_path, std::vector<DoorDef>& out, std::string& err);
  bool parseDoorsString(const std::string& s, std::vector<DoorDef>& out, std::string& err);

  // hot-reload
  bool reloadDoorsIfRequested();

  void ensureDoorIO(const DoorDef& d);

  // tf/radius
  bool getRobotXY(double& x, double& y);
  bool inRadius(const DoorDef& d, double rx, double ry) const;

  // geometry
  static bool segmentsIntersect(double p1x, double p1y, double p2x, double p2y,
                                double q1x, double q1y, double q2x, double q2y);
  static int orientation(double ax, double ay, double bx, double by, double cx, double cy);
  static bool onSegment(double ax, double ay, double bx, double by, double cx, double cy);

  // selection
  bool pathUsesDoor(const nav_msgs::msg::Path& plan, const DoorDef& d) const;
  bool cooldownPassed(const std::string& ns) const;

  // cmd/feedback
  void publishCmd(const std::string& ns, bool open);
  bool doorOpened(const DoorDef& d);
  bool doorClosed(const DoorDef& d);
};

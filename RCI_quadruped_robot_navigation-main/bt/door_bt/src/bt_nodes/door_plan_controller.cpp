#include "door_bt/bt_nodes/door_plan_controller.hpp"

#include <tf2/exceptions.h>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <cctype>

static std::string trim_copy(std::string x)
{
  auto issp = [](int c){ return std::isspace(c); };
  x.erase(x.begin(), std::find_if(x.begin(), x.end(), [&](int c){ return !issp(c); }));
  x.erase(std::find_if(x.rbegin(), x.rend(), [&](int c){ return !issp(c); }).base(), x.end());
  return x;
}

DoorPlanController::DoorPlanController(const std::string& name,
                                       const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("bt_door_plan_controller");
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList DoorPlanController::providedPorts()
{
  return {
    BT::InputPort<std::string>("doors", "legacy: inline doors string"),
    BT::InputPort<std::string>("doors_yaml", "YAML path (preferred). If set, overrides 'doors'"),

    BT::InputPort<std::string>("plan_topic", "default: /plan"),
    BT::InputPort<std::string>("world_frame", "default: map"),
    BT::InputPort<std::string>("base_frame", "default: base_link"),

    BT::InputPort<bool>("require_feedback", "default: true"),
    BT::InputPort<double>("open_threshold", "default: 0.05"),
    BT::InputPort<double>("close_threshold", "default: 0.02"),
    BT::InputPort<double>("feedback_timeout_s", "default: 5.0"),
    BT::InputPort<double>("cooldown_s", "default: 1.0"),

    // runtime control (optional)
    BT::InputPort<std::string>("config_yaml_topic", "default: /door_controller/config_yaml"),
    BT::InputPort<std::string>("enabled_topic", "default: /door_controller/enabled")
  };
}

void DoorPlanController::planCb(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  last_plan_ = msg;
}

void DoorPlanController::doorPosCb(const std::string& ns,
                                   const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  auto& io = io_[ns];
  io.has_pos = true;

  double m = 0.0;
  for (auto v : msg->data) m = std::max(m, std::fabs(v));
  io.pos_abs_max = m;
  io.last_pos_time = node_->now();
}

void DoorPlanController::configYamlCb(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(cfg_mtx_);
  pending_yaml_ = msg->data;
  reload_requested_.store(true);
}

void DoorPlanController::enabledCb(const std_msgs::msg::Bool::SharedPtr msg)
{
  enabled_ = msg->data;
}

std::string DoorPlanController::resolvePathMaybeInShare(const std::string& path) const
{
  if (path.empty()) return path;
  if (!path.empty() && path[0] == '/') return path;  // abs

  try
  {
    auto share = ament_index_cpp::get_package_share_directory("door_bt");
    return share + "/" + path;
  }
  catch (...)
  {
    return path; 
  }
}

bool DoorPlanController::loadDoorsFromYaml(const std::string& yaml_path,
                                           std::vector<DoorDef>& out,
                                           std::string& err)
{
  err.clear();
  out.clear();

  const std::string full = resolvePathMaybeInShare(yaml_path);

  try
  {
    YAML::Node root = YAML::LoadFile(full);
    if (!root["doors"])
    {
      err = "YAML missing key: doors";
      return false;
    }

    for (const auto& n : root["doors"])
    {
      if (!n["ns"] || !n["p1"] || !n["p2"] || !n["c"] || !n["radius"])
      {
        err = "YAML door entry missing fields (ns,p1,p2,c,radius)";
        return false;
      }

      DoorDef d;
      d.ns = n["ns"].as<std::string>();
      auto p1 = n["p1"]; auto p2 = n["p2"]; auto c = n["c"];

      d.p1x = p1[0].as<double>(); d.p1y = p1[1].as<double>();
      d.p2x = p2[0].as<double>(); d.p2y = p2[1].as<double>();
      d.cx  = c[0].as<double>();  d.cy  = c[1].as<double>();
      d.radius = n["radius"].as<double>();

      out.push_back(d);
    }

    if (out.empty())
    {
      err = "YAML doors list is empty";
      return false;
    }

    RCLCPP_INFO(node_->get_logger(), "Loaded %zu doors from YAML: %s", out.size(), full.c_str());
    return true;
  }
  catch (const std::exception& e)
  {
    err = std::string("YAML exception: ") + e.what();
    return false;
  }
}

bool DoorPlanController::parseDoorsString(const std::string& s,
                                          std::vector<DoorDef>& out,
                                          std::string& err)
{
  out.clear();
  err.clear();

  std::stringstream ss(s);
  std::string item;

  while (std::getline(ss, item, ';'))
  {
    item = trim_copy(item);
    if (item.empty()) continue;

    std::vector<std::string> toks;
    std::stringstream ss2(item);
    std::string tok;
    while (std::getline(ss2, tok, ','))
      toks.push_back(trim_copy(tok));

    // ns + 7 numbers
    if (toks.size() != 8)
    {
      err = "doors format error: need 8 tokens (ns + 7 numbers)";
      return false;
    }

    DoorDef d;
    d.ns = toks[0];
    try
    {
      d.p1x = std::stod(toks[1]); d.p1y = std::stod(toks[2]);
      d.p2x = std::stod(toks[3]); d.p2y = std::stod(toks[4]);
      d.cx  = std::stod(toks[5]); d.cy  = std::stod(toks[6]);
      d.radius = std::stod(toks[7]);
    }
    catch (...)
    {
      err = "doors format error: numeric parse failed";
      return false;
    }

    out.push_back(d);
  }

  if (out.empty())
  {
    err = "doors is empty";
    return false;
  }
  return true;
}

void DoorPlanController::ensureDoorIO(const DoorDef& d)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (io_.count(d.ns)) return;

  DoorIO io;
  io.pub_cmd = node_->create_publisher<std_msgs::msg::Bool>(d.ns + "/door_open", 10);

  io.sub_pos = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    d.ns + "/door_pos", rclcpp::QoS(10),
    [this, ns=d.ns](const std_msgs::msg::Float64MultiArray::SharedPtr msg){
      this->doorPosCb(ns, msg);
    });

  io_[d.ns] = io;

  rt_.try_emplace(d.ns, DoorRuntime{});

  RCLCPP_INFO(node_->get_logger(),
    "Door IO ready: %s (open=%s, pos=%s)",
    d.ns.c_str(), (d.ns + "/door_open").c_str(), (d.ns + "/door_pos").c_str());
}

bool DoorPlanController::reloadDoorsIfRequested()
{
  if (!reload_requested_.exchange(false))
    return false;

  std::string new_yaml;
  {
    std::lock_guard<std::mutex> lk(cfg_mtx_);
    new_yaml = pending_yaml_;
  }

  new_yaml = trim_copy(new_yaml);
  if (new_yaml.empty())
    return false;
  if (new_yaml == doors_yaml_)
    return false;

  std::vector<DoorDef> new_doors;
  std::string err;
  if (!loadDoorsFromYaml(new_yaml, new_doors, err))
  {
    RCLCPP_ERROR(node_->get_logger(), "failed to reload doors_yaml [%s]: %s",
                 new_yaml.c_str(), err.c_str());
    return false;
  }

  // swap config
  doors_yaml_ = new_yaml;
  doors_ = std::move(new_doors);

  for (const auto& d : doors_)
  {
    ensureDoorIO(d);
    rt_[d.ns] = DoorRuntime{};
  }

  processed_in_plan_.clear();
  done_time_.clear();
  last_plan_stamp_ = rclcpp::Time(0,0,node_->get_clock()->get_clock_type());
  last_plan_size_ = 0;

  RCLCPP_INFO(node_->get_logger(), "Door config reloaded: %s (doors=%zu)",
              doors_yaml_.c_str(), doors_.size());
  return true;
}

bool DoorPlanController::getRobotXY(double& x, double& y)
{
  rclcpp::spin_some(node_);
  try
  {
    auto tf = tf_buffer_->lookupTransform(world_frame_, base_frame_, tf2::TimePointZero);
    x = tf.transform.translation.x;
    y = tf.transform.translation.y;
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "TF lookup failed (%s)", ex.what());
    return false;
  }
}

bool DoorPlanController::inRadius(const DoorDef& d, double rx, double ry) const
{
  const double dx = rx - d.cx;
  const double dy = ry - d.cy;
  return (dx*dx + dy*dy) <= d.radius * d.radius;
}

int DoorPlanController::orientation(double ax, double ay, double bx, double by, double cx, double cy)
{
  double val = (by - ay) * (cx - bx) - (bx - ax) * (cy - by);
  const double eps = 1e-9;
  if (std::fabs(val) < eps) return 0;
  return (val > 0.0) ? 1 : 2;
}

bool DoorPlanController::onSegment(double ax, double ay, double bx, double by, double cx, double cy)
{
  const double eps = 1e-9;
  return (bx <= std::max(ax, cx) + eps &&
          bx + eps >= std::min(ax, cx) &&
          by <= std::max(ay, cy) + eps &&
          by + eps >= std::min(ay, cy));
}

bool DoorPlanController::segmentsIntersect(
  double p1x, double p1y, double p2x, double p2y,
  double q1x, double q1y, double q2x, double q2y)
{
  int o1 = orientation(p1x, p1y, p2x, p2y, q1x, q1y);
  int o2 = orientation(p1x, p1y, p2x, p2y, q2x, q2y);
  int o3 = orientation(q1x, q1y, q2x, q2y, p1x, p1y);
  int o4 = orientation(q1x, q1y, q2x, q2y, p2x, p2y);

  if (o1 != o2 && o3 != o4) return true;

  if (o1 == 0 && onSegment(p1x, p1y, q1x, q1y, p2x, p2y)) return true;
  if (o2 == 0 && onSegment(p1x, p1y, q2x, q2y, p2x, p2y)) return true;
  if (o3 == 0 && onSegment(q1x, q1y, p1x, p1y, q2x, q2y)) return true;
  if (o4 == 0 && onSegment(q1x, q1y, p2x, p2y, q2x, q2y)) return true;

  return false;
}

bool DoorPlanController::pathUsesDoor(const nav_msgs::msg::Path& plan, const DoorDef& d) const
{
  if (plan.poses.size() < 2) return false;
  for (size_t i = 0; i + 1 < plan.poses.size(); ++i)
  {
    const auto& a = plan.poses[i].pose.position;
    const auto& b = plan.poses[i+1].pose.position;
    if (segmentsIntersect(a.x, a.y, b.x, b.y, d.p1x, d.p1y, d.p2x, d.p2y))
      return true;
  }
  return false;
}

bool DoorPlanController::cooldownPassed(const std::string& ns) const
{
  auto it = done_time_.find(ns);
  if (it == done_time_.end()) return true;
  return (node_->now() - it->second).seconds() >= cooldown_s_;
}

void DoorPlanController::publishCmd(const std::string& ns, bool open)
{
  auto& rt = rt_[ns];
  if (rt.has_last_cmd && rt.last_cmd_open == open)
    return;

  std_msgs::msg::Bool msg;
  msg.data = open;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = io_.find(ns);
    if (it == io_.end() || !it->second.pub_cmd)
    {
      RCLCPP_ERROR(node_->get_logger(), "publisher missing ns=[%s]", ns.c_str());
      return;
    }
    it->second.pub_cmd->publish(msg);
  }

  rt.has_last_cmd = true;
  rt.last_cmd_open = open;
  rt.last_cmd_time = node_->now();

  RCLCPP_INFO(node_->get_logger(), "%s -> %s", ns.c_str(), open ? "OPEN" : "CLOSE");
}

bool DoorPlanController::doorOpened(const DoorDef& d)
{
  std::lock_guard<std::mutex> lk(mtx_);
  auto it = io_.find(d.ns);
  if (it == io_.end() || !it->second.has_pos) return false;
  return it->second.pos_abs_max >= open_threshold_;
}

bool DoorPlanController::doorClosed(const DoorDef& d)
{
  std::lock_guard<std::mutex> lk(mtx_);
  auto it = io_.find(d.ns);
  if (it == io_.end() || !it->second.has_pos) return false;
  return it->second.pos_abs_max <= close_threshold_;
}

BT::NodeStatus DoorPlanController::onStart()
{
  (void)getInput("plan_topic", plan_topic_);
  (void)getInput("world_frame", world_frame_);
  (void)getInput("base_frame", base_frame_);
  (void)getInput("require_feedback", require_feedback_);
  (void)getInput("open_threshold", open_threshold_);
  (void)getInput("close_threshold", close_threshold_);
  (void)getInput("feedback_timeout_s", feedback_timeout_s_);
  (void)getInput("cooldown_s", cooldown_s_);
  (void)getInput("doors_yaml", doors_yaml_);
  (void)getInput("config_yaml_topic", config_yaml_topic_);
  (void)getInput("enabled_topic", enabled_topic_);

  std::string err;

  if (!doors_yaml_.empty())
  {
    if (!loadDoorsFromYaml(doors_yaml_, doors_, err))
    {
      RCLCPP_ERROR(node_->get_logger(), "failed to load doors_yaml: %s", err.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    std::string doors_str;
    if (!getInput("doors", doors_str))
    {
      RCLCPP_ERROR(node_->get_logger(), "missing [doors] (or set doors_yaml)");
      return BT::NodeStatus::FAILURE;
    }
    if (!parseDoorsString(doors_str, doors_, err))
    {
      RCLCPP_ERROR(node_->get_logger(), "doors parse error: %s", err.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  sub_plan_ = node_->create_subscription<nav_msgs::msg::Path>(
    plan_topic_, rclcpp::QoS(1),
    std::bind(&DoorPlanController::planCb, this, std::placeholders::_1));

  // runtime control subscriptions
  sub_config_yaml_ = node_->create_subscription<std_msgs::msg::String>(
    config_yaml_topic_, rclcpp::QoS(10),
    std::bind(&DoorPlanController::configYamlCb, this, std::placeholders::_1));

  sub_enabled_ = node_->create_subscription<std_msgs::msg::Bool>(
    enabled_topic_, rclcpp::QoS(10),
    std::bind(&DoorPlanController::enabledCb, this, std::placeholders::_1));

  for (const auto& d : doors_) ensureDoorIO(d);

  processed_in_plan_.clear();
  last_plan_stamp_ = rclcpp::Time(0,0,node_->get_clock()->get_clock_type());
  last_plan_size_ = 0;

  RCLCPP_INFO(node_->get_logger(),
    "DoorPlanController started (CONCURRENT): plan=%s world=%s base=%s doors=%zu (cfg_topic=%s enabled_topic=%s)",
    plan_topic_.c_str(),
    world_frame_.c_str(),
    base_frame_.c_str(),
    doors_.size(),
    config_yaml_topic_.c_str(),
    enabled_topic_.c_str());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DoorPlanController::onRunning()
{
  rclcpp::spin_some(node_);

  (void)reloadDoorsIfRequested();

  if (!enabled_)
  {
    return BT::NodeStatus::RUNNING;
  }

  nav_msgs::msg::Path::SharedPtr plan;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    plan = last_plan_;
  }

  double rx=0, ry=0;
  if (!getRobotXY(rx, ry))
    return BT::NodeStatus::RUNNING;

  if (plan && plan->poses.size() >= 2)
  {
    if (plan->header.stamp != last_plan_stamp_ || plan->poses.size() != last_plan_size_)
    {
      processed_in_plan_.clear();
      last_plan_stamp_ = plan->header.stamp;
      last_plan_size_ = plan->poses.size();
    }
  }

  for (const auto& d : doors_)
  {
    auto& rt = rt_[d.ns];

    const bool inside = inRadius(d, rx, ry);

    if ((rt.opened || rt.opening) && !inside)
    {
      if (!rt.closing)
      {
        rt.closing = true;
        rt.opening = false;  
        rt.opened = true;    
        rt.has_last_cmd = false; 
      }

      if (!require_feedback_)
      {
        publishCmd(d.ns, false);
        rt.opened = false;
        rt.closing = false;
        processed_in_plan_.insert(d.ns);
        done_time_[d.ns] = node_->now();
        continue;
      }

      if (doorClosed(d))
      {
        rt.opened = false;
        rt.closing = false;
        processed_in_plan_.insert(d.ns);
        done_time_[d.ns] = node_->now();
        continue;
      }

      publishCmd(d.ns, false);

      if ((node_->now() - rt.last_cmd_time).seconds() > feedback_timeout_s_)
      {
        rt.opened = false;
        rt.closing = false;
        processed_in_plan_.insert(d.ns);
        done_time_[d.ns] = node_->now();
      }

      continue;
    }

    if (inside)
    {
      if (processed_in_plan_.count(d.ns)) continue;
      if (!cooldownPassed(d.ns)) continue;

      if (!plan || plan->poses.size() < 2) continue;
      if (!pathUsesDoor(*plan, d)) continue;

      if (!require_feedback_)
      {
        if (!rt.opened)
          publishCmd(d.ns, true);
        rt.opened = true;
        rt.opening = false;
        rt.closing = false;
        continue;
      }

      if (doorOpened(d))
      {
        rt.opened = true;
        rt.opening = false;
        rt.closing = false;
        continue;
      }

      if (!rt.opened && !rt.opening)
      {
        rt.opening = true;
        rt.closing = false;
        rt.has_last_cmd = false; 
      }

      publishCmd(d.ns, true);

      if ((node_->now() - rt.last_cmd_time).seconds() > feedback_timeout_s_)
      {
        rt.opened = true;
        rt.opening = false;
      }

      continue;
    }

  }

  return BT::NodeStatus::RUNNING;
}

void DoorPlanController::onHalted()
{
}

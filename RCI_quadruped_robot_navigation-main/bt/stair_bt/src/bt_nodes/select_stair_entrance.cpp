#include "stair_bt/bt_nodes/select_stair_entrance.hpp"

#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <regex>
#include <cmath>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace stair_bt
{

static double normalize_angle(double a)
{
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

SelectStairEntrance::SelectStairEntrance(
    const std::string & name,
    const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
  node_ = std::make_shared<rclcpp::Node>("select_stair_entrance");
}

void SelectStairEntrance::fillPose(geometry_msgs::msg::PoseStamped & pose,
                                   const std::string & frame,
                                   double x, double y, double yaw)
{
  pose.header.frame_id = frame;
  pose.header.stamp = node_->now();

  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
}

BT::NodeStatus SelectStairEntrance::tick()
{
  int current = 0, target = 0;

  {
    auto res1 = getInput<int>("current_floor", current);
    auto res2 = getInput<int>("target_floor", target);
    if (!res1 || !res2)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[SelectStairEntrance] current_floor/target_floor 입력 없음");
      return BT::NodeStatus::FAILURE;
    }
  }

  // direction
  std::string direction;
  getInput<std::string>("direction", direction);
  if (direction.empty())
  {
    direction = (target > current) ? "up" : "down";
  }

  // yaml path
  std::string yaml_path;
  if (!getInput<std::string>("stair_yaml", yaml_path))
    yaml_path.clear();

  if (yaml_path.empty() || yaml_path.find("$(") != std::string::npos)
  {
    try {
      auto share_dir = ament_index_cpp::get_package_share_directory("stair_bt");
      yaml_path = share_dir + "/config/stairs.yaml";
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[SelectStairEntrance] 패키지 share 디렉토리 조회 실패: %s", e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  std::string map_frame = "map";
  getInput<std::string>("map_frame", map_frame);

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[SelectStairEntrance] YAML 로드 실패: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }

  YAML::Node stairs = root["stairs"];
  if (!stairs) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[SelectStairEntrance] 'stairs' 노드 없음");
    return BT::NodeStatus::FAILURE;
  }

  std::string chosen_id;
  YAML::Node chosen_node;

  for (auto it : stairs)
  {
    std::string key = it.first.as<std::string>();
    std::regex re("^stair_(\\d+)_(\\d+)$");
    std::smatch m;
    if (!std::regex_match(key, m, re))
      continue;

    int f1 = std::stoi(m[1].str());
    int f2 = std::stoi(m[2].str());

    if ((f1 == current && f2 == target) ||
        (f1 == target && f2 == current))
    {
      chosen_id = key;
      chosen_node = it.second;
      break;
    }
  }

  if (!chosen_node || chosen_id.empty())
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "[SelectStairEntrance] %d -> %d 에 해당하는 stair_* 없음",
                 current, target);
    return BT::NodeStatus::FAILURE;
  }

  YAML::Node entry = chosen_node["entry"];
  if (!entry)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "[SelectStairEntrance] %s 에 entry 블록 없음",
                 chosen_id.c_str());
    return BT::NodeStatus::FAILURE;
  }

  YAML::Node entry_dir = entry;
  if (entry["up"] || entry["down"])
  {
    if (entry[direction])
    {
      entry_dir = entry[direction];
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[SelectStairEntrance] %s entry.%s 없음 (x,y가 다르면 반드시 필요)",
                   chosen_id.c_str(), direction.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  if (!entry_dir["x"] || !entry_dir["y"])
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "[SelectStairEntrance] %s entry.%s x/y 없음",
                 chosen_id.c_str(), direction.c_str());
    return BT::NodeStatus::FAILURE;
  }

  double x = entry_dir["x"].as<double>();
  double y = entry_dir["y"].as<double>();

  double yaw = 0.0;
  if (entry_dir["yaw"])
  {
    yaw = entry_dir["yaw"].as<double>();
  }
  else
  {
    if (entry["up"] && entry["up"]["yaw"])
    {
      double up_yaw = entry["up"]["yaw"].as<double>();
      yaw = (direction == "down") ? normalize_angle(up_yaw + M_PI) : normalize_angle(up_yaw);
      RCLCPP_WARN(node_->get_logger(),
                  "[SelectStairEntrance] %s entry.%s yaw 없음 -> up yaw로부터 생성 (확실하지 않음)",
                  chosen_id.c_str(), direction.c_str());
    }
    else if (entry["yaw"])
    {
      double base_yaw = entry["yaw"].as<double>();
      yaw = (direction == "down") ? normalize_angle(base_yaw + M_PI) : normalize_angle(base_yaw);
      RCLCPP_WARN(node_->get_logger(),
                  "[SelectStairEntrance] %s entry yaw만 존재 -> direction으로 yaw 생성 (확실하지 않음)",
                  chosen_id.c_str());
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[SelectStairEntrance] %s entry yaw 정보가 전혀 없음",
                   chosen_id.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  geometry_msgs::msg::PoseStamped pose;
  fillPose(pose, map_frame, x, y, yaw);


  double turn1 = 0.0, move_d = 0.0, turn2 = 0.0;
  YAML::Node landing = chosen_node["landing"];
  if (!landing)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "[SelectStairEntrance] %s landing 블록 없음",
                 chosen_id.c_str());
    return BT::NodeStatus::FAILURE;
  }

  YAML::Node landing_dir = landing;
  bool used_up_as_down = false;

  if (landing["up"] || landing["down"])
  {
    if (landing[direction])
    {
      landing_dir = landing[direction];
      used_up_as_down = false;
    }
    else if (direction == "down" && landing["up"])
    {
      landing_dir = landing["up"];
      used_up_as_down = true;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[SelectStairEntrance] %s landing.%s 없음",
                   chosen_id.c_str(), direction.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  if (!landing_dir["turn1_deg"] || !landing_dir["move_d"] || !landing_dir["turn2_deg"])
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "[SelectStairEntrance] %s landing(turn1/move/turn2) 값 부족",
                 chosen_id.c_str());
    return BT::NodeStatus::FAILURE;
  }

  turn1  = landing_dir["turn1_deg"].as<double>();
  move_d = landing_dir["move_d"].as<double>();
  turn2  = landing_dir["turn2_deg"].as<double>();

  if (direction == "down" && used_up_as_down)
  {
    turn1 = -turn1;
    turn2 = -turn2;
  }


  double dz_mid = 2.5;
  double dz_top = 5.0;

  if (chosen_node["mid"] && chosen_node["mid"]["dz_from_entry"])
    dz_mid = chosen_node["mid"]["dz_from_entry"].as<double>();
  if (chosen_node["top"] && chosen_node["top"]["dz_from_entry"])
    dz_top = chosen_node["top"]["dz_from_entry"].as<double>();


  double flight_up = 0.5, flight_down = 0.5;
  double land_up   = 0.5, land_down   = 0.5;

  YAML::Node sp = chosen_node["speed"];
  if (sp)
  {
    if (sp["flight"])
    {
      if (sp["flight"]["up"])   flight_up   = sp["flight"]["up"].as<double>();
      if (sp["flight"]["down"]) flight_down = sp["flight"]["down"].as<double>();
    }
    if (sp["landing"])
    {
      if (sp["landing"]["up"])   land_up   = sp["landing"]["up"].as<double>();
      if (sp["landing"]["down"]) land_down = sp["landing"]["down"].as<double>();
    }
  }

  const double flight_speed  = (direction == "down") ? flight_down : flight_up;
  const double landing_speed = (direction == "down") ? land_down   : land_up;

  // outputs
  setOutput<std::string>("stair_id", chosen_id);
  setOutput<geometry_msgs::msg::PoseStamped>("entry_pose", pose);

  setOutput<double>("entry_yaw", yaw);
  setOutput<double>("landing_turn1_deg", turn1);
  setOutput<double>("landing_move_d", move_d);
  setOutput<double>("landing_turn2_deg", turn2);

  setOutput<double>("dz_mid", dz_mid);
  setOutput<double>("dz_top", dz_top);
  setOutput<double>("flight_speed", std::abs(flight_speed));
  setOutput<double>("landing_speed", std::abs(landing_speed));

  RCLCPP_INFO(node_->get_logger(),
              "[SelectStairEntrance] %d->%d dir=%s stair_id=%s "
              "entry=(%.2f,%.2f,yaw=%.3f) landing=(%.1f,%.2f,%.1f) "
              "dz(mid=%.2f, top=%.2f) speed(flight=%.2f, landing=%.2f)",
              current, target, direction.c_str(), chosen_id.c_str(),
              x, y, yaw, turn1, move_d, turn2,
              dz_mid, dz_top, flight_speed, landing_speed);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace stair_bt

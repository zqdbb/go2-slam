#pragma once
#include <string>
#include <yaml-cpp/yaml.h>

namespace elevator_bt {

struct ElevatorZonesConfig
{
  double zone_min_x{0.0}, zone_min_y{0.0};
  double zone_max_x{0.0}, zone_max_y{0.0};

  double cabin_min_x{0.0}, cabin_min_y{0.0};
  double cabin_max_x{0.0}, cabin_max_y{0.0};

  double front_offset{0.4};   
  double rear_offset{-0.4};   
};

inline bool load_zones_config(const std::string& yaml_path, ElevatorZonesConfig& out)
{
  try
  {
    YAML::Node root = YAML::LoadFile(yaml_path);
    if (!root["zones"]) {
      return false;
    }
    auto zones = root["zones"];

    // elevator zone
    auto zmin = zones["zone_min"];
    auto zmax = zones["zone_max"];
    out.zone_min_x = zmin[0].as<double>();
    out.zone_min_y = zmin[1].as<double>();
    out.zone_max_x = zmax[0].as<double>();
    out.zone_max_y = zmax[1].as<double>();

    // cabin box
    auto cmin = zones["cabin_min"];
    auto cmax = zones["cabin_max"];
    out.cabin_min_x = cmin[0].as<double>();
    out.cabin_min_y = cmin[1].as<double>();
    out.cabin_max_x = cmax[0].as<double>();
    out.cabin_max_y = cmax[1].as<double>();

    if (root["robot"])
    {
      auto robot = root["robot"];
      if (robot["front_offset"]) {
        out.front_offset = robot["front_offset"].as<double>();
      }
      if (robot["rear_offset"]) {
        out.rear_offset = robot["rear_offset"].as<double>();
      }
    }

    return true;
  }
  catch (const std::exception& e)
  {
    (void)e;
    return false;
  }
}

} // namespace elevator_bt

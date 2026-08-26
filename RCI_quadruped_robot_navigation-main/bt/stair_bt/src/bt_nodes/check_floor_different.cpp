#include "stair_bt/bt_nodes/check_floor_different.hpp"

#include <rclcpp/rclcpp.hpp>

namespace stair_bt
{

BT::NodeStatus CheckFloorDifferent::tick()
{
  int current = 0;
  int target = 0;

  if (!getInput<int>("current_floor", current) ||
      !getInput<int>("target_floor", target))
  {
    return BT::NodeStatus::FAILURE;
  }

  int diff = target - current;

  if (diff == 0) {
    setOutput<int>("floors_to_climb", 0);
    setOutput<std::string>("direction", "none");
    return BT::NodeStatus::FAILURE;  
  }

  int steps = std::abs(diff);
  std::string dir = (diff > 0) ? "up" : "down";

  setOutput<int>("floors_to_climb", steps);
  setOutput<std::string>("direction", dir);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace stair_bt

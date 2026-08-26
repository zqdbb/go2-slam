#include <behaviortree_cpp_v3/bt_factory.h>

#include "stair_bt/bt_nodes/turn_relative.hpp"
#include "stair_bt/bt_nodes/align_with_stairs.hpp"
#include "stair_bt/bt_nodes/move_forward_distance.hpp"
#include "stair_bt/bt_nodes/climb_flight_rl.hpp"

#include "stair_bt/bt_nodes/get_target_floor_from_topic.hpp"
#include "stair_bt/bt_nodes/check_floor_different.hpp"
#include "stair_bt/bt_nodes/select_stair_entrance.hpp"
#include "stair_bt/bt_nodes/nav2_navigate_to_pose.hpp"
#include "stair_bt/bt_nodes/compute_next_floor.hpp"

using namespace stair_bt;

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<TurnRelative>("TurnRelative");
  factory.registerNodeType<AlignWithStairs>("AlignWithStairs");
  factory.registerNodeType<MoveForwardDistance>("MoveForwardDistance");
  factory.registerNodeType<ClimbFlightRL>("ClimbFlightRL");

  factory.registerNodeType<GetTargetFloorFromTopic>("GetTargetFloorFromTopic");
  factory.registerNodeType<CheckFloorDifferent>("CheckFloorDifferent");
  factory.registerNodeType<SelectStairEntrance>("SelectStairEntrance");
  factory.registerNodeType<Nav2NavigateToPose2>("Nav2NavigateToPose2");
  factory.registerNodeType<ComputeNextFloor>("ComputeNextFloor");

}

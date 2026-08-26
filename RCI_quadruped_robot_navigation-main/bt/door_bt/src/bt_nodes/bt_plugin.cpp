#include <behaviortree_cpp_v3/bt_factory.h>
#include "door_bt/bt_nodes/door_plan_controller.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<DoorPlanController>("DoorPlanController"); 

}
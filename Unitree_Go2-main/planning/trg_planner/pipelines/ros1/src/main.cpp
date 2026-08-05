#include <ros/ros.h>

#include "ros1_node.h"

int main(int argc, char **argv) {
  signal(SIGINT, signal_handler);  // to exit program when ctrl+c

  ros::init(argc, argv, "trg_ros1_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("~");
  ROS1Node        trg_ros1_node(nh);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}

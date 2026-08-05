#ifndef PIPELINES_ROS1_INCLUDE_ROS1_UTILS_HPP_
#define PIPELINES_ROS1_INCLUDE_ROS1_UTILS_HPP_

#pragma once

#include "trg_planner/include/utils/common.h"

//// ROS1
#include <ros/package.h>
#include <ros/ros.h>

//// std
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

//// geometry
#include <geometry_msgs/PoseStamped.h>

//// sensor_msgs
#include <sensor_msgs/PointCloud2.h>

//// nav_msgs
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

//// visualization
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//// Grid Map
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>

#include "grid_map_ros/grid_map_ros.hpp"

//// TF
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace ROS1Types {
using Pose          = geometry_msgs::PoseStamped;
using Odom          = nav_msgs::Odometry;
using PointCloud    = sensor_msgs::PointCloud2;
using PointCloudPtr = sensor_msgs::PointCloud2Ptr;
using GridMap       = grid_map_msgs::GridMap;
using MarkerArray   = visualization_msgs::MarkerArray;
using Marker        = visualization_msgs::Marker;
using Path          = nav_msgs::Path;
using FloatArray    = std_msgs::Float32MultiArray;
}  // namespace ROS1Types

inline void publishCloud(const std::string frame_id, PointCloudPtr& cloud, ros::Publisher& pub) {
  ROS1Types::PointCloud cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.stamp    = ros::Time::now();
  cloud_msg.header.frame_id = frame_id;
  cloud_msg.height          = 1;
  cloud_msg.width           = cloud->size();
  pub.publish(cloud_msg);
}

inline void publishPath(const std::string                   frame_id,
                        const std::vector<Eigen::Vector3f>& path,
                        ros::Publisher&                     pub) {
  ROS1Types::Path path_msg;
  path_msg.header.stamp    = ros::Time::now();
  path_msg.header.frame_id = frame_id;
  for (const auto& pt : path) {
    ROS1Types::Pose p;
    p.pose.position.x = pt.x();
    p.pose.position.y = pt.y();
    p.pose.position.z = pt.z();
    path_msg.poses.push_back(p);
  }
  pub.publish(path_msg);
}

#endif  // PIPELINES_ROS1_INCLUDE_ROS1_UTILS_HPP_

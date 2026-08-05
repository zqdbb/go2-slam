#ifndef PIPELINES_ROS2_INCLUDE_ROS2_UTILS_HPP_
#define PIPELINES_ROS2_INCLUDE_ROS2_UTILS_HPP_

#pragma once

#include "trg_planner/include/utils/common.h"

//// ROS1
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

//// std
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

//// geometry
#include <geometry_msgs/msg/pose_stamped.hpp>

//// sensor_msgs
#include <sensor_msgs/msg/point_cloud2.hpp>

//// nav_msgs
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

//// visualization
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

//// TF2
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

//// PCL
#include <pcl_conversions/pcl_conversions.h>

namespace ROS2Types {
using Pose          = geometry_msgs::msg::PoseStamped;
using Odom          = nav_msgs::msg::Odometry;
using PointCloud    = sensor_msgs::msg::PointCloud2;
using PointCloudPtr = sensor_msgs::msg::PointCloud2::SharedPtr;
using MarkerArray   = visualization_msgs::msg::MarkerArray;
using Marker        = visualization_msgs::msg::Marker;
using Path          = nav_msgs::msg::Path;
using FloatArray    = std_msgs::msg::Float32MultiArray;
}  // namespace ROS2Types

inline void publishCloud(const rclcpp::Node::SharedPtr&                      node,
                         const std::string                                   frame_id,
                         PointCloudPtr&                                      cloud,
                         rclcpp::Publisher<ROS2Types::PointCloud>::SharedPtr pub) {
  ROS2Types::PointCloud cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.stamp    = node->now();
  cloud_msg.header.frame_id = frame_id;
  cloud_msg.height          = 1;
  cloud_msg.width           = cloud->size();
  pub->publish(cloud_msg);
}

inline void publishPath(const rclcpp::Node::SharedPtr&                node,
                        const std::string                             frame_id,
                        const std::vector<Eigen::Vector3f>&           path,
                        rclcpp::Publisher<ROS2Types::Path>::SharedPtr pub) {
  ROS2Types::Path path_msg;
  path_msg.header.stamp    = node->now();
  path_msg.header.frame_id = frame_id;
  for (const auto& pt : path) {
    ROS2Types::Pose p;
    p.pose.position.x = pt.x();
    p.pose.position.y = pt.y();
    p.pose.position.z = pt.z();
    path_msg.poses.push_back(p);
  }
  pub->publish(path_msg);
}

#endif  // PIPELINES_ROS2_INCLUDE_ROS2_UTILS_HPP_

/**
 * Copyright 2025, Korea Advanced Institute of Science and Technology
 * Massachusetts Institute of Technology,
 * Daejeon, 34051
 * All Rights Reserved
 * Authors: Dongkyu Lee, et al.
 * See LICENSE for the license information
 */
#ifndef COMMON_H_
#define COMMON_H_

#pragma once

#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <atomic>
#include <cassert>
#include <chrono>
#include <deque>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
//// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//// DataType
using PtsDefault    = pcl::PointXYZ;
using PointCloudPtr = std::shared_ptr<pcl::PointCloud<PtsDefault>>;

//// Handler
inline std::atomic<bool> is_running(true);
inline void              signal_handler(int signal) {
  is_running.store(false);
  std::cout << "\033[1;38m====================================\033[0m" << std::endl;
  std::cout << "\033[1;31mYou pressed Ctrl + C, exiting\033[0m" << std::endl;  // red
  std::cout << "\033[1;33mYou pressed Ctrl + C, exiting\033[0m" << std::endl;  // yellow
  std::cout << "\033[1;32mYou pressed Ctrl + C, exiting\033[0m" << std::endl;  // green
  std::cout << "\033[1;34mYou pressed Ctrl + C, exiting\033[0m" << std::endl;  // blue
  std::cout << "\033[1;35mYou pressed Ctrl + C, exiting\033[0m" << std::endl;  // magenta
  std::cout << "\033[1;36mYou pressed Ctrl + C, exiting\033[0m" << std::endl;  // cyan
  std::cout << "\033[1;37mYou pressed Ctrl + C, exiting\033[0m" << std::endl;  // white
  std::cout << "\033[1;38m====================================\033[0m" << std::endl;
  exit(signal);
}

/// Timer
inline std::chrono::high_resolution_clock::time_point tic() {
  return std::chrono::high_resolution_clock::now();
}

inline float toc(std::chrono::high_resolution_clock::time_point start_time,
                 const std::string                              unit = "ms") {
  auto  end_time = std::chrono::high_resolution_clock::now();
  float elapsed_time;

  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
  if (unit == "ns") {
    elapsed_time = duration.count();  // [ns]
  } else if (unit == "us") {
    elapsed_time = duration.count() * 1e-3;  // [us]
  } else if (unit == "ms") {
    elapsed_time = duration.count() * 1e-6;  // [ms]
  } else if (unit == "s") {
    elapsed_time = duration.count() * 1e-9;  // [s]
  } else {
    elapsed_time = duration.count() * 1e-6;  // default [ms]
  }

  return elapsed_time;
}

/// Print
inline void print(const std::string &msg, bool isVerbose = true) {
  if (isVerbose) {
    std::cout << msg << std::endl;
  }
}

inline void print_error(const std::string &msg, bool isVerbose = true) {
  if (isVerbose) {
    std::cout << "\033[1;31m" << msg << "\033[0m" << std::endl;  // red
  }
}

inline void print_warning(const std::string &msg, bool isVerbose = true) {
  if (isVerbose) {
    std::cout << "\033[1;33m" << msg << "\033[0m" << std::endl;  // yellow
  }
}

inline void print_success(const std::string &msg, bool isVerbose = true) {
  if (isVerbose) {
    std::cout << "\033[1;32m" << msg << "\033[0m" << std::endl;  // green
  }
}

inline std::string to_string_float(float num, int decimal = 2) {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(decimal)
     << std::round(num * std::pow(10, decimal)) / std::pow(10, decimal);
  return ss.str();
}

/// Convert
inline void EigenToPointCloud(const Eigen::MatrixXf &mat, PointCloudPtr &cloud) {
  cloud->clear();
  for (int i = 0; i < mat.rows(); ++i) {
    PtsDefault pt;
    pt.x = mat(i, 0);
    pt.y = mat(i, 1);
    pt.z = mat(i, 2);
    cloud->push_back(pt);
  }
}

inline Eigen::MatrixXf PointCloudToEigen(const PointCloudPtr &cloud) {
  Eigen::MatrixXf mat(cloud->size(), 3);
  int             i = 0;
  for (const auto &pt : cloud->points) {
    mat.row(i) << pt.x, pt.y, pt.z;
    i++;
  }
  return mat;
}

#endif  // COMMON_H_

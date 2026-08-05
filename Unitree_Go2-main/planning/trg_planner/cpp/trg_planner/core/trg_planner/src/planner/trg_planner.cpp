/**
 * Copyright 2025, Korea Advanced Institute of Science and Technology
 * Massachusetts Institute of Technology,
 * Daejeon, 34051
 * All Rights Reserved
 * Authors: Dongkyu Lee, et al.
 * See LICENSE for the license information
 */
#include "trg_planner/include/planner/trg_planner.h"

TRGPlanner::TRGPlanner() {}
TRGPlanner::~TRGPlanner() {
  thd.graph.join();
  thd.planning.join();
}

void TRGPlanner::init() {
  //// Load TRG
  trg_ = std::make_shared<TRG>(param_.isVerbose,
                               param_.expandDist,
                               param_.robotSize,
                               param_.sampleNum,
                               param_.heightThreshold,
                               param_.collisionThreshold,
                               param_.updateCollisionThreshold,
                               param_.safetyFactor,
                               param_.goal_tolerance);

  if (trg_ == nullptr) {
    print_error("Failed to initialize TRG");
    exit(1);
  }

  //// Load prebuilt map
  if (param_.isPreMap) {
    cs_.preMapPtr.reset(new pcl::PointCloud<PtsDefault>());
    cs_.preMapPtr->clear();
    loadPrebuiltMap();
    if (cs_.preMapPtr == nullptr) {
      print_error("Failed to load prebuilt map");
      exit(1);
    }
    trg_->setGlobalMap(cs_.preMapPtr);
  }

  //// Initialize observation map
  cs_.obsPtr.reset(new pcl::PointCloud<PtsDefault>());

  //// TODO: Load prebuilt graph
  if (param_.isPreGraph) {
    print("Prebuilt graph is loaded");
  } else {
    print("Prebuilt graph is not loaded");
  }

  //// Initialize FSM Threads
  thd.graph    = std::thread(&TRGPlanner::runGraphFSM, this);
  thd.planning = std::thread(&TRGPlanner::runPlanningFSM, this);
}

void TRGPlanner::loadPrebuiltMap() {
  std::string map_type = param_.preMapPath.substr(param_.preMapPath.find_last_of(".") + 1);
  if (map_type != "pcd") {
    print_error("Unsupported map type");
    exit(1);
  }

  std::string                 abs_path = std::string(TRG_DIR) + "/../../" + param_.preMapPath;
  pcl::PointCloud<PtsDefault> rawMap   = pcl::PointCloud<PtsDefault>();
  if (pcl::io::loadPCDFile<PtsDefault>(abs_path, rawMap) == -1) {
    print_error("Failed to load prebuilt map");
    exit(1);
  }

  if (param_.isVoxelize) {
    pcl::VoxelGrid<PtsDefault> vg;
    vg.setInputCloud(rawMap.makeShared());
    vg.setLeafSize(param_.VoxelSize, param_.VoxelSize, param_.VoxelSize);
    vg.filter(*cs_.preMapPtr);
  } else {
    *cs_.preMapPtr = rawMap;
  }
  print("Prebuilt map size: " + std::to_string(rawMap.size()) + " -> " +
        std::to_string(cs_.preMapPtr->size()));
  print("Prebuilt map is loaded");
}

void TRGPlanner::setParams(const std::string& config_path) {
  print("Loading config from: " + config_path);

  YAML::Node config = YAML::LoadFile(config_path);

  param_.isVerbose = config["isVerbose"].as<bool>(true);

  param_.graph_rate    = config["timer"]["graphRate"].as<float>(1.0f);
  param_.planning_rate = config["timer"]["planningRate"].as<float>(1.0f);

  param_.isPreMap   = config["map"]["isPrebuiltMap"].as<bool>(false);
  param_.preMapPath = config["map"]["prebuiltMapPath"].as<std::string>("");
  param_.isVoxelize = config["map"]["isVoxelize"].as<bool>(false);
  param_.VoxelSize  = config["map"]["voxelSize"].as<float>(0.1f);

  param_.isPreGraph               = config["trg"]["isPrebuiltTRG"].as<bool>(false);
  param_.preGraphPath             = config["trg"]["prebuiltTRGPath"].as<std::string>("");
  param_.isUpdate                 = config["trg"]["isUpdate"].as<bool>(false);
  param_.expandDist               = config["trg"]["expandDist"].as<float>(0.6f);
  param_.robotSize                = config["trg"]["robotSize"].as<float>(0.3f);
  param_.sampleNum                = config["trg"]["sampleNum"].as<int>(20);
  param_.heightThreshold          = config["trg"]["heightThreshold"].as<float>(0.15f);
  param_.collisionThreshold       = config["trg"]["collisionThreshold"].as<float>(0.2f);
  param_.updateCollisionThreshold = config["trg"]["updateCollisionThreshold"].as<float>(0.2f);
  param_.safetyFactor             = config["trg"]["safetyFactor"].as<float>(1.0f);
  param_.goal_tolerance           = config["trg"]["goalTolerance"].as<float>(0.8f);
}

void TRGPlanner::runGraphFSM() {
  while (is_running.load()) {
    auto start_loop = tic();
    std::this_thread::sleep_for(std::chrono::nanoseconds(1));

    fsm_.graph.notice();
    switch (fsm_.graph.curr_state_) {
      case graphState::INIT: {
        if (flag_.graphInit) {
          fsm_.graph.transition(graphState::UPDATE);
          break;
        }

        if (param_.isPreMap) {
          auto start_init_graph = tic();
          trg_->initGraph(param_.isPreMap, state_.pose3d);
          print_warning("Graph initialization time: " + std::to_string(toc(start_init_graph, "s")) +
                        " sec");
          flag_.graphInit = true;
          fsm_.graph.transition(graphState::UPDATE);
          break;
        }

        if (!flag_.poseIn || !flag_.obsIn) {
          print_warning("Pose: " + std::to_string(flag_.poseIn) +
                        ", Obs: " + std::to_string(flag_.obsIn));
          fsm_.graph.transition(graphState::INIT);
          break;
        }
        mtx.obs.lock();
        trg_->setGlobalMap(cs_.obsPtr);
        mtx.obs.unlock();

        auto start_init_graph = tic();
        trg_->initGraph(param_.isPreMap, state_.pose3d);
        print_warning("Graph initialization time: " + std::to_string(toc(start_init_graph, "s")) +
                      " sec");
        flag_.graphInit = true;
        fsm_.graph.transition(graphState::UPDATE);
        break;
      }
      case graphState::UPDATE: {
        if (param_.isUpdate) {
          if (!flag_.poseIn || !flag_.obsIn) {
            print_warning("Pose: " + std::to_string(flag_.poseIn) +
                          ", Obs: " + std::to_string(flag_.obsIn));
            fsm_.graph.transition(graphState::UPDATE);
            break;
          }
          auto start = tic();
          mtx.obs.lock();  // too slow, and not necessary (then, comment this line)
          trg_->setLocalMap(state_.pose2d, cs_.obsPtr);
          if (!param_.isPreMap) trg_->setGlobalMap(cs_.obsPtr);
          mtx.obs.unlock();
          trg_->updateGraph();
          print_success("Graph update time: " + std::to_string(toc(start, "ms")) + " ms");
        }
        fsm_.graph.transition(graphState::UPDATE);
        break;
      }
      case graphState::LOAD: {
        break;
      }
      case graphState::RESET: {
        break;
      }
      case graphState::SAVE: {
        break;
      }
      default: {
        print_error("Invalid graph state");
        exit(1);
      }
    }
    float loop_time   = toc(start_loop, "ms");
    int   remain_time = 1000 / param_.graph_rate - loop_time;
    if (remain_time > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(remain_time));
    }
    thd.hz["graph"] = std::round(1000 / toc(start_loop, "ms") * 100) / 100;
  }
}

void TRGPlanner::runPlanningFSM() {
  while (is_running.load()) {
    auto start_loop = tic();
    std::this_thread::sleep_for(std::chrono::nanoseconds(1));

    if (flag_.goalIn) {
      flag_.goalIn = false;
      fsm_.planning.transition(planningState::PLANNING);
    }
    fsm_.planning.notice();
    switch (fsm_.planning.curr_state_) {
      case planningState::RESET: {
        flag_.pathFound = false;
        path_.clear();
        fsm_.planning.transition(planningState::RESET);
        break;
      }
      case planningState::PLANNING: {
        path_.clear();
        auto start = tic();
        if (trg_->planSafePath(state_.pose2d,
                               goal_state_.pose,
                               path_.raw,
                               path_.direct_dist,
                               path_.raw_path_length,
                               path_.avg_risk)) {
          path_.planning_time = toc(start, "ms");
          print_success("Path planning time: " + std::to_string(path_.planning_time) + " ms");
          flag_.pathFound     = true;
          flag_.planningCount = 0;
          trg_->refinePath(path_.raw, path_.smooth);
          fsm_.planning.transition(planningState::ONGOING);
        } else {
          print_error("Failed to find a path, count: " + std::to_string(flag_.planningCount));
          flag_.planningCount++;
          if (flag_.planningCount > 10) {
            fsm_.planning.transition(planningState::RESET);
          } else {
            fsm_.planning.transition(planningState::PLANNING);
          }
        }
        break;
      }
      case planningState::ONGOING: {
        if (trg_->checkReadched(state_.pose2d)) {
          print_success("Goal reached");
          fsm_.planning.transition(planningState::RESET);
          break;
        }
        if (trg_->checkReplan(state_.pose2d, path_.raw)) {
          print_warning("Replanning");
          fsm_.planning.transition(planningState::PLANNING);
          break;
        }
        fsm_.planning.transition(planningState::ONGOING);
        break;
      }
      default: {
        print_error("Invalid planning state");
        exit(1);
      }
    }
    float loop_time   = toc(start_loop, "ms");
    int   remain_time = 1000 / param_.planning_rate - loop_time;
    if (remain_time > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(remain_time));
    }
    thd.hz["planning"] = std::round(1000 / toc(start_loop, "ms") * 100) / 100;
  }
}

void GraphFSM::transition(graphState new_state) {
  prev_state_ = curr_state_;
  curr_state_ = new_state;
  if (prev_state_ != curr_state_) {
    print("[Graph] " + state_map_[prev_state_] + " -> " + state_map_[curr_state_]);
  }
}

void GraphFSM::notice() {
  if (prev_state_ != curr_state_) {
    print("[Graph] " + state_map_[curr_state_]);
  }
}

void PlanningFSM::transition(planningState new_state) {
  prev_state_ = curr_state_;
  curr_state_ = new_state;
  if (prev_state_ != curr_state_) {
    print("[Planning] " + state_map_[prev_state_] + " -> " + state_map_[curr_state_]);
  }
}

void PlanningFSM::notice() {
  if (prev_state_ != curr_state_) {
    print("[Planning] " + state_map_[curr_state_]);
  }
}

std::shared_ptr<TRG> TRGPlanner::getTRG() { return trg_; }

void TRGPlanner::setPose(const Eigen::Vector3f& pose     = Eigen::Vector3f::Zero(),
                         const Eigen::Vector4f& quat     = Eigen::Vector4f(1, 0, 0, 0),
                         const std::string&     frame_id = "map") {
  std::lock_guard<std::mutex> lock(mtx.odom);
  state_.frame_id = frame_id;
  state_.pose3d   = pose;
  state_.pose2d   = pose.head(2);
  state_.quat     = quat;
  Eigen::Quaternionf q(state_.quat[0], state_.quat[1], state_.quat[2], state_.quat[3]);
  state_.T_B2M.block<3, 3>(0, 0) = q.toRotationMatrix();
  state_.T_B2M.block<3, 1>(0, 3) = state_.pose3d;
  flag_.poseIn                   = true;
}

void TRGPlanner::setObs(const Eigen::MatrixXf& obs) {
  if (!flag_.poseIn) {
    print_error("Pose is not initialized");
    return;
  }
  std::lock_guard<std::mutex> lock(mtx.obs);
  cs_.obsPtr->clear();
  for (int i = 0; i < obs.rows(); i++) {
    PtsDefault pt;
    pt.x = obs(i, 0);
    pt.y = obs(i, 1);
    pt.z = obs(i, 2);
    cs_.obsPtr->push_back(pt);
  }
  flag_.obsIn = true;
}

void TRGPlanner::setGoal(const Eigen::Vector3f& pose,
                         const Eigen::Vector4f& quat = Eigen::Vector4f(1, 0, 0, 0)) {
  if (!flag_.graphInit) {
    print_error("Graph is not initialized");
    return;
  }
  std::lock_guard<std::mutex> lock(mtx.goal);
  goal_state_.pose = pose;
  goal_state_.quat = quat;
  goal_state_.init = true;
  flag_.goalIn     = true;
}

std::vector<Eigen::Vector3f> TRGPlanner::getPlannedPath(const std::string& type) {
  if (!flag_.pathFound) {
    print_error("Path is not found");
    return {};
  }
  std::vector<Eigen::Vector3f> path;
  if (type == "raw") {
    for (auto& pt : path_.raw) {
      path.push_back(pt.head(3));
    }
  } else if (type == "smooth") {
    for (auto& pt : path_.smooth) {
      path.push_back(pt.head(3));
    }
  } else {
    print_error("Invalid path type");
    return {};
  }
  return path;
}

std::vector<float> TRGPlanner::getPathInfo() {
  if (!flag_.pathFound) {
    print_error("Path is not found");
    return {};
  }
  std::vector<float> info;
  info.push_back(path_.direct_dist);
  info.push_back(path_.raw_path_length);
  info.push_back(path_.smooth_path_length);
  info.push_back(path_.planning_time);
  info.push_back(path_.avg_risk);
  return info;
}

Eigen::MatrixXf TRGPlanner::getMapEigen(const std::string& type = "pre") {
  if (type == "pre") {
    if (!param_.isPreMap) {
      print_error("Prebuilt map is not loaded");
      return Eigen::MatrixXf();
    }
    return PointCloudToEigen(cs_.preMapPtr);
  } else if (type == "obs") {
    return PointCloudToEigen(cs_.obsPtr);
  } else {
    print_error("Invalid map type");
    return Eigen::MatrixXf();
  }
}

Eigen::Vector3f TRGPlanner::getGoalPose() {
  if (!goal_state_.init) {
    print_error("Goal is not initialized");
    return Eigen::Vector3f::Zero();
  }
  return goal_state_.pose;
}

Eigen::Vector4f TRGPlanner::getGoalQuat() {
  if (!goal_state_.init) {
    print_error("Goal is not initialized");
    return Eigen::Vector4f::Zero();
  }
  return goal_state_.quat;
}

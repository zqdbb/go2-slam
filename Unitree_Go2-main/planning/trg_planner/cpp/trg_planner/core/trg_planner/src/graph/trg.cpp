/**
 * Copyright 2025, Korea Advanced Institute of Science and Technology
 * Massachusetts Institute of Technology,
 * Daejeon, 34051
 * All Rights Reserved
 * Authors: Dongkyu Lee, et al.
 * See LICENSE for the license information
 */
#include "trg_planner/include/graph/trg.h"

TRG::TRG(bool  isVerbose,
         float expand_dist,
         float robot_size,
         int   sample_num,
         float height_threshold,
         float collision_threshold,
         float update_collision_threshold,
         float safety_factor,
         float goal_tolerance)
    : gen_(rd_()), distr_(0.0, 1.0) {
  param_.isVerbose                  = isVerbose;
  param_.expand_dist                = expand_dist;
  param_.robot_size                 = robot_size;
  param_.sample_num                 = sample_num;
  param_.height_threshold           = height_threshold;
  param_.collision_threshold        = collision_threshold;
  param_.update_collision_threshold = update_collision_threshold;
  param_.safety_factor              = safety_factor;
  param_.goal_tolerance             = goal_tolerance;
  this->resetGraph("global");
  this->resetGraph("local");
  this->resetMap("global");
  this->resetMap("local");
}

void TRG::initGraph(bool isPreMap, Eigen::Vector3f start3d = Eigen::Vector3f::Zero()) {
  std::lock_guard<std::mutex> lock(mtx.graph);
  print("TRG initGraph", param_.isVerbose);
  trgStruct& graph = *trgMap_["global"];
  this->resetGraph(graph.type);

  assert(graph.cloud_map->size() > 0 && "Map is empty");

  graph.root_pos           = start3d.head(2);
  Eigen::Vector2f root_pos = graph.root_pos;
  root_pos.x()             = root_pos.x() + param_.expand_dist;
  int cnt                  = 0;
  while (!this->addNode(graph.node_id, root_pos, NodeState::Valid, graph.type)) {
    if (cnt > 100) {
      print_error("Failed to generate root node");
      exit(1);
    }
    root_pos = root_pos + Eigen::Vector2f(param_.expand_dist * distr_(gen_),
                                          param_.expand_dist * distr_(gen_));
    cnt++;
  }

  assert(graph.nodes.size() > 0 && "Graph is empty");

  this->expandGraph(graph.node_id - 1, graph.type);
  this->cleanGraph(false);

  assert(graph.node_id == 1 && "Root node is not generated");
}

void TRG::loadPrebuiltGraph() {
  //// TODO: load prebuilt graph
  print("TRG loadPrebuiltGraph");
}

void TRG::setGlobalMap(PointCloudPtr& map) {
  print("TRG setGlobalMap", param_.isVerbose);
  trgStruct& global_graph = *trgMap_["global"];
  this->resetMap("global");
  *global_graph.cloud_map = *map;

  for (int i = 0; i < global_graph.cloud_map->size(); ++i) {
    PtsDefault& pt = global_graph.cloud_map->points[i];
    kd_insert2(global_graph.map_tree, pt.x, pt.y, &pt);
  }

  print("Root pos: " + std::to_string(global_graph.root_pos.x()) + ", " +
            std::to_string(global_graph.root_pos.y()),
        param_.isVerbose);
}

void TRG::setLocalMap(Eigen::Vector2f start2d, PointCloudPtr& map) {
  std::lock_guard<std::mutex> lock(mtx.graph);
  trgStruct&                  local_graph = *trgMap_["local"];
  this->resetMap("local");

  local_graph.root_pos   = start2d;
  *local_graph.cloud_map = *map;

  for (int i = 0; i < local_graph.cloud_map->size(); ++i) {
    PtsDefault& pt = local_graph.cloud_map->points[i];
    kd_insert2(local_graph.map_tree, pt.x, pt.y, &pt);
  }

  this->setLocalGraph(false);
}

void TRG::setLocalGraph(bool useMutex = false) {
  if (useMutex) {
    std::lock_guard<std::mutex> lock(mtx.graph);
  }
  trgStruct& global_graph = *trgMap_["global"];
  trgStruct& local_graph  = *trgMap_["local"];
  this->resetGraph("local");
  for (auto& node : global_graph.nodes) {
    kdres* res = kd_nearest_range2(local_graph.map_tree,
                                   node.second->pos_.x(),
                                   node.second->pos_.y(),
                                   param_.robot_size * 0.5);
    if (kd_res_size(res) == 0) {
      kd_res_free(res);
      continue;
    }
    local_graph.nodes[node.first] = node.second;
    kd_insert2(local_graph.node_tree, node.second->pos_.x(), node.second->pos_.y(), node.second);
    kd_res_free(res);
  }
}

bool TRG::addNode(int node_id, Eigen::Vector2f& node_pos, NodeState state, std::string type) {
  trgStruct& graph = *trgMap_[type];

  /// Check reference position is valid
  if (node_id == 0) {
    if (this->isCollision(node_pos, graph.type, param_.collision_threshold)) {
      return false;
    }
  }

  /// Create new node
  kdres*      res = kd_nearest2(graph.map_tree, node_pos.x(), node_pos.y());
  PtsDefault* pt  = reinterpret_cast<PtsDefault*>(kd_res_item_data(res));
  kd_res_free(res);
  Node* node           = new Node(node_id, node_pos, pt->z, state);
  graph.nodes[node_id] = node;
  kd_insert2(graph.node_tree, node_pos.x(), node_pos.y(), node);
  graph.node_id++;
  return true;
}

void TRG::wireEdge(Node* node1, Node* node2, std::string type) {
  if (node1->id_ == node2->id_) {
    return;
  }
  for (auto& edge : node1->edges_) {
    if (edge->dst_id_ == node2->id_) {
      return;
    }
  }
  for (auto& edge : node2->edges_) {
    if (edge->dst_id_ == node1->id_) {
      return;
    }
  }

  float max_slope = atan2(param_.height_threshold, param_.robot_size);
  float slope     = atan2(fabs(node1->pos_.z() - node2->pos_.z()),
                      (node1->pos_.head(2) - node2->pos_.head(2)).norm());
  if (slope > max_slope) {
    return;
  }

  float           dist   = (node1->pos_.head(2) - node2->pos_.head(2)).norm();
  Eigen::Vector2f dir    = (node2->pos_.head(2) - node1->pos_.head(2)).normalized();
  Eigen::Vector2f center = node1->pos_.head(2) + 0.5 * dist * dir;
  assert(dist < 2.5 * param_.expand_dist && "Edge is too Long");

  /// check collision between node1 and node2
  float ds = param_.robot_size * 0.5;
  for (float i = 0; i < dist; i += ds) {
    Eigen::Vector2f pos = node1->pos_.head(2) + i * dir;
    if (this->isCollision(pos, type, param_.collision_threshold)) {
      return;
    }
  }

  /// get Elipse points with focus at node1 and node2
  float c = 0.5 * dist;         // focus distance
  float b = param_.robot_size;  // minor axis
  float a = b;                  // if c < b, ellipse is circle
  if (c >= b) {                 // if c >= b, ellipse is ellipse
    a = sqrt(c * c + b * b);    // major axis
  }
  bool isCircle = (a == b) ? true : false;

  /// get circle points with center at center point and radius of a
  trgStruct&                       graph = *trgMap_[type];
  pcl::PointCloud<PtsDefault>::Ptr ellipse_pts(new pcl::PointCloud<PtsDefault>());
  Eigen::Matrix2f                  R;        // rotation matrix for TF from global to local
  R << dir.x(), -dir.y(), dir.y(), dir.x();  // (dir is x-axis)
  kdres* res = kd_nearest_range2(graph.map_tree, center.x(), center.y(), a);
  if (kd_res_size(res) == 0) {
    kd_res_free(res);
    return;
  }
  while (!kd_res_end(res)) {
    PtsDefault*     pt = reinterpret_cast<PtsDefault*>(kd_res_item_data(res));
    PtsDefault      p;
    Eigen::Vector2f p2d(pt->x - center.x(), pt->y - center.y());
    p2d = R * p2d;
    p.x = p2d.x();
    p.y = p2d.y();
    p.z = pt->z;
    if (isCircle) {
      ellipse_pts->push_back(p);
    } else {
      if ((p.x * p.x) * (b * b) + (p.y * p.y) * (a * a) < a * a * b * b) {
        ellipse_pts->push_back(p);
      }
    }
    kd_res_next(res);
  }
  kd_res_free(res);
  if (ellipse_pts->size() < 3) {
    return;
  }

  /// get normal vector of the ellipse
  int             n = ellipse_pts->size();
  Eigen::MatrixXf A(n, 3);
  for (int i = 0; i < n; i++) {
    A.row(i) << ellipse_pts->points[i].x, ellipse_pts->points[i].y, ellipse_pts->points[i].z;
  }
  Eigen::MatrixXf centered = A.rowwise() - A.colwise().mean();
  Eigen::MatrixXf cov      = (centered.adjoint() * centered) / static_cast<double>(A.rows() - 1);
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
  Eigen::MatrixXf                   eigenvectors = svd.matrixU().normalized();
  Eigen::Vector3f                   normal       = eigenvectors.col(2);

  if (normal.z() < 0) {
    normal = -normal;
  }

  float hor_grad = eigenvectors.col(0).dot(-gravity);
  float ver_grad = eigenvectors.col(1).dot(-gravity);
  if (hor_grad < 0) {
    hor_grad = (-eigenvectors.col(0)).dot(-gravity);
  }
  if (ver_grad < 0) {
    ver_grad = (-eigenvectors.col(1)).dot(-gravity);
  }

  assert(hor_grad >= 0 && "Horizontal gradient is negative");
  assert(ver_grad >= 0 && "Vertical gradient is negative");

  float ratio  = 0.8;
  float weight = ratio * hor_grad + (1 - ratio) * ver_grad;
  if (weight < 0.1) {
    weight = 0.0;
  }

  Edge* edge_1 = new Edge(node2->id_, weight, dist);
  Edge* edge_2 = new Edge(node1->id_, weight, dist);
  node1->edges_.push_back(edge_1);
  node2->edges_.push_back(edge_2);
  return;
}

void TRG::expandGraph(int ref_id, std::string type) {
  trgStruct& graph    = *trgMap_[type];
  Node*      ref_node = graph.nodes.at(ref_id);

  std::deque<Node*> expand_queue;
  expand_queue.push_back(ref_node);
  while (!expand_queue.empty()) {
    // std::this_thread::sleep_for(std::chrono::milliseconds(5)); // delay for visualization
    Node* node = expand_queue.front();
    expand_queue.pop_front();

    /// Sampling
    std::vector<Eigen::Vector2f> samples;
    int                          max_trial_sample = 1000;
    int                          trial_sample     = 0;
    while (samples.size() < param_.sample_num) {
      if (trial_sample > max_trial_sample) {
        break;
      }
      // 0.75 param_.expand_dist ~ 1.25 param_.expand_dist
      // float expand_dist = 0.75 * param_.expand_dist + distr_(gen_) * (1.25 * param_.expand_dist -
      // 0.75 * param_.expand_dist); // 0.75 ~ 1.25
      float           expand_dist = param_.expand_dist;
      float           angle       = distr_(gen_) * 2 * M_PI;
      Eigen::Vector2f sample =
          node->pos_.head(2) + Eigen::Vector2f(expand_dist * cos(angle), expand_dist * sin(angle));
      if (this->isCollision(sample, graph.type, param_.collision_threshold)) {
        trial_sample++;
        continue;
      }
      samples.push_back(sample);
    }

    /// Add new nodes and wire edges
    for (auto& sample : samples) {
      /// 1. Check if the sample is already in the graph
      kdres* res           = kd_nearest2(graph.node_tree, sample.x(), sample.y());
      Node*  existing_node = reinterpret_cast<Node*>(kd_res_item_data(res));
      kd_res_free(res);
      if (existing_node->state_ == NodeState::Invalid) {
        continue;
      }
      if ((existing_node->pos_.head(2) - sample).norm() < param_.robot_size) {
        this->wireEdge(node, existing_node, graph.type);
        continue;
      }

      /// 2. Add new node
      NodeState new_state = (ref_id == 0) ? NodeState::Valid : NodeState::Frontier;
      if (!this->addNode(graph.node_id, sample, new_state, graph.type)) {
        continue;
      }
      Node* new_node = graph.nodes.at(graph.node_id - 1);
      this->wireEdge(
          node, new_node, graph.type);  /// 2.1 Wire edge between new node and reference node

      /// 3. Wire edge between new node and existing nodes
      if (param_.expand_dist - param_.robot_size < 0.25 * param_.expand_dist) {
        kdres* res2 = kd_nearest_range2(
            graph.node_tree, new_node->pos_.x(), new_node->pos_.y(), param_.expand_dist);
        if (kd_res_size(res2) > 0) {
          while (!kd_res_end(res2)) {
            Node* existing_node = reinterpret_cast<Node*>(kd_res_item_data(res2));
            if (existing_node->state_ == NodeState::Invalid) {
              kd_res_next(res2);
              continue;
            }
            this->wireEdge(new_node, existing_node, graph.type);
            kd_res_next(res2);
          }
        }
        kd_res_free(res2);
      }

      /// 4. Add new node to expand queue
      if (new_node->edges_.size() < 1) {
        new_node->state_ = NodeState::Invalid;
        continue;
      }
      expand_queue.push_back(new_node);
    }
  }
}

void TRG::updateGraph() {
  // print("TRG updateGraph", param_.isVerbose);
  std::lock_guard<std::mutex> lock(mtx.graph);

  trgStruct& global_graph = *trgMap_["global"];
  trgStruct& local_graph  = *trgMap_["local"];

  std::deque<Node*> expand_queue;
  for (auto& node : local_graph.nodes) {
    Eigen::Vector2f npos2d = node.second->pos_.head(2);
    if ((npos2d - local_graph.root_pos).norm() > 2.0 * param_.expand_dist) {
      if (this->isCollision(npos2d, local_graph.type, param_.update_collision_threshold) ||
          node.second->edges_.size() < 1) {
        node.second->state_ = NodeState::Invalid;
        continue;
      }
    }

    if (this->isFrontier(npos2d) && node.second->state_ == NodeState::Frontier) {
      node.second->state_ = NodeState::Frontier;
      expand_queue.push_back(node.second);
      continue;
    }
    expand_queue.push_back(node.second);
    node.second->state_ = NodeState::Valid;
  }

  while (!expand_queue.empty()) {
    Node* node = expand_queue.front();
    expand_queue.pop_front();
    this->expandGraph(node->id_, global_graph.type);
  }
  this->cleanGraph(true);
}

void TRG::cleanGraph(bool updateLocal = true) {
  trgStruct&                     global_graph = *trgMap_["global"];
  std::unordered_map<int, int>   old2new;
  std::unordered_map<int, Node*> new_nodes;
  std::vector<int>               del_edges;
  int                            new_id = 0;
  for (auto& node : global_graph.nodes) {
    if (node.second->state_ == NodeState::Invalid || node.second->edges_.size() < 1) {
      continue;
    }
    node.second->id_    = new_id;
    new_nodes[new_id]   = node.second;
    old2new[node.first] = new_id;
    new_id++;
    for (auto& edge : node.second->edges_) {
      if (global_graph.nodes[edge->dst_id_]->state_ == NodeState::Invalid) {
        del_edges.push_back(edge->dst_id_);
      }
    }
  }

  for (auto& node : new_nodes) {
    std::vector<Edge*> new_edges;
    for (auto& edge : node.second->edges_) {
      if (std::find(del_edges.begin(), del_edges.end(), edge->dst_id_) != del_edges.end()) {
        continue;
      }
      Edge* new_edge = new Edge(old2new[edge->dst_id_], edge->weight_, edge->dist_);
      new_edges.push_back(new_edge);
    }
    node.second->edges_.clear();
    node.second->edges_ = new_edges;
  }

  this->resetGraph(global_graph.type);
  global_graph.nodes   = new_nodes;
  global_graph.node_id = new_id;
  for (auto& node : global_graph.nodes) {
    kd_insert2(global_graph.node_tree, node.second->pos_.x(), node.second->pos_.y(), node.second);
  }

  if (updateLocal) {
    this->setLocalGraph(false);
  }
}

void TRG::setGoal(Eigen::Vector3f& goal) {
  // print("TRG setGoal", param_.isVerbose);
  trgStruct& global_graph = *trgMap_["global"];

  goal_.pose3d = goal;
  goal_.pose2d = goal.head(2);

  kdres* res = kd_nearest_range2(global_graph.node_tree, goal.x(), goal.y(), param_.robot_size);
  if (kd_res_size(res) == 0) {
    kd_res_free(res);
    float min_dist = std::numeric_limits<float>::max();
    for (auto& node : global_graph.nodes) {
      // if (node.second->state_ != NodeState::Frontier) {
      //     continue;
      // }
      float dist = (node.second->pos_.head(2) - goal.head(2)).norm();
      if (dist < min_dist) {
        min_dist   = dist;
        goal_.node = node.second;
      }
    }
    goal_.isKnown = false;
  } else {
    Node* near_node = reinterpret_cast<Node*>(kd_res_item_data(res));
    goal_.node      = near_node;
    kd_res_free(res);
    goal_.isKnown = true;
  }
}

bool TRG::checkReadched(Eigen::Vector2f& pos2d) {
  // print("TRG checkReached", param_.isVerbose);
  float dist = (goal_.pose2d - pos2d).norm();
  if (dist < param_.goal_tolerance) {
    return true;
  }
  return false;
}

bool TRG::checkReplan(Eigen::Vector2f& pos2d, std::vector<Eigen::Vector3f>& path) {
  // print("TRG checkReplan", param_.isVerbose);
  if (goal_.node == nullptr) {
    return false;
  }

  float dist2subgoal = (goal_.node->pos_.head(2) - pos2d).norm();
  if (!goal_.isKnown && dist2subgoal < param_.goal_tolerance) {
    return true;
  }

  if (!goal_.isKnown && goal_.node->state_ != NodeState::Frontier) {
    return true;
  }

  trgStruct& global_graph = *trgMap_["global"];
  for (auto& pt : path) {
    kdres* res = kd_nearest_range2(global_graph.node_tree, pt.x(), pt.y(), param_.robot_size);
    if (kd_res_size(res) == 0) {
      kd_res_free(res);
      return true;
    }
    kd_res_free(res);
  }
  return false;
}

bool TRG::planSafePath(Eigen::Vector2f&              start2d,
                       Eigen::Vector3f&              goal_pose,
                       std::vector<Eigen::Vector3f>& out_path,
                       float&                        direct_dist,
                       float&                        path_length,
                       float&                        avg_risk) {
  // print("TRG planSafePath", param_.isVerbose);
  std::lock_guard<std::mutex> lock(mtx.graph);
  this->setGoal(goal_pose);

  trgStruct& global_graph = *trgMap_["global"];

  kdres* res        = kd_nearest2(global_graph.node_tree, start2d.x(), start2d.y());
  Node*  start_node = reinterpret_cast<Node*>(kd_res_item_data(res));

  std::priority_queue<OptimizeNode*,
                      std::vector<OptimizeNode*>,
                      std::function<bool(OptimizeNode*, OptimizeNode*)>>
      open_list([](OptimizeNode* a, OptimizeNode* b) { return a->f_ > b->f_; });
  std::vector<OptimizeNode*> open_check(global_graph.nodes.size(), nullptr);

  direct_dist                = (goal_.node->pos_.head(2) - start_node->pos_.head(2)).norm();
  double        g_cost       = 0.0;
  double        f_cost       = g_cost + direct_dist;
  OptimizeNode* st_opti_node = new OptimizeNode(start_node->id_, f_cost, g_cost);
  st_opti_node->parent_      = nullptr;
  open_list.push(st_opti_node);
  open_check[st_opti_node->id_] = st_opti_node;

  std::vector<OptimizeNode*> close_list(global_graph.nodes.size(), nullptr);

  while (!open_list.empty()) {
    OptimizeNode* opti_node = open_list.top();
    open_list.pop();
    open_check[opti_node->id_] = nullptr;

    if (opti_node->id_ == goal_.node->id_) {
      OptimizeNode* node       = opti_node;
      float         sum_dist   = 0.0;
      float         sum_weight = 0.0;
      float         avg_weight = 0.0;
      while (node != nullptr) {
        Node* n = global_graph.nodes.at(node->id_);
        for (auto& edge : n->edges_) {
          if (node->parent_ != nullptr && edge->dst_id_ == node->parent_->id_) {
            sum_dist += edge->dist_;
            sum_weight += edge->weight_;
            break;
          }
        }
        out_path.push_back(n->pos_);
        node = node->parent_;
      }
      avg_weight = sum_weight / out_path.size();
      std::reverse(out_path.begin(), out_path.end());
      path_length = sum_dist;
      avg_risk    = avg_weight;
      // print_success("Path found [dist: " + std::to_string(sum_dist) +
      //               ", risk: " + std::to_string(avg_weight) + "]");
      return true;
    }

    Node* curr_node            = global_graph.nodes.at(opti_node->id_);
    close_list[curr_node->id_] = opti_node;

    for (auto e : curr_node->edges_) {
      Node* dst_node = global_graph.nodes.at(e->dst_id_);
      if (close_list[dst_node->id_] != nullptr || dst_node->state_ == NodeState::Invalid) {
        continue;
      }

      double next_g_cost = opti_node->g_ + (param_.safety_factor * e->weight_ + 1) * e->dist_;
      double next_f_cost = next_g_cost + (goal_.node->pos_.head(2) - dst_node->pos_.head(2)).norm();

      OptimizeNode* dst_opti_node = new OptimizeNode(dst_node->id_, next_f_cost, next_g_cost);
      dst_opti_node->parent_      = opti_node;

      if (open_check[dst_node->id_] == nullptr) {
        open_list.push(dst_opti_node);
        open_check[dst_node->id_] = dst_opti_node;
      } else if (dst_opti_node->g_ < open_check[dst_node->id_]->g_) {
        open_list.push(dst_opti_node);
        open_check[dst_node->id_] = dst_opti_node;
      }
    }
  }
  return false;
}

void TRG::refinePath(std::vector<Eigen::Vector3f>& in_path,
                     std::vector<Eigen::Vector3f>& out_path) {
  std::deque<Eigen::Vector3f> dense_path;
  int                         point_between = 1;
  for (int i = 0; i < in_path.size() - 1; ++i) {
    Eigen::Vector3f p1   = in_path[i];
    Eigen::Vector3f p2   = in_path[i + 1];
    Eigen::Vector3f dir  = (p2.head(2) - p1.head(2)).normalized();
    float           dist = (p2.head(2) - p1.head(2)).norm();
    dense_path.push_back(p1);
    for (int j = 1; j < point_between; ++j) {
      Eigen::Vector3f p;
      p.head(2) = p1.head(2) + j * dist / point_between * dir;
      p.z()     = p1.z() + j * (p2.z() - p1.z()) / point_between;
      dense_path.push_back(p);
    }
    dense_path.push_back(p2);
  }

  std::deque<Eigen::Vector3f> smooth_path;
  for (int i = 0; i < dense_path.size(); ++i) {
    if (i == dense_path.size() - 1) {
      smooth_path.push_back(dense_path[i]);
      break;
    }
    Eigen::Vector3f sum(0.0, 0.0, 0.0);
    int             cnt = 0;
    for (int j = i - 1; j < i + 2; ++j) {
      if (j < 0 || j >= dense_path.size()) {
        continue;
      }
      sum += dense_path[j];
      cnt++;
    }
    Eigen::Vector3f avg = sum / cnt;
    smooth_path.push_back(avg);
  }
  out_path = std::vector<Eigen::Vector3f>(smooth_path.begin(), smooth_path.end());
}

void TRG::resetGraph(std::string type) {
  trgStruct& graph = *trgMap_[type];
  graph.nodes.clear();
  kd_clear(graph.node_tree);
  graph.node_id = 0;
}

void TRG::resetMap(std::string type) {
  trgStruct& graph = *trgMap_[type];
  kd_clear(graph.map_tree);
  graph.cloud_map.reset(new pcl::PointCloud<PtsDefault>());
  graph.cloud_map->clear();
}

bool TRG::isCollision(Eigen::Vector2f& pos, std::string type, float threshold = 0.1) {
  trgStruct& graph = *trgMap_[type];
  kdres*     res   = kd_nearest_range2(graph.map_tree, pos.x(), pos.y(), param_.robot_size);
  if (kd_res_size(res) == 0) {
    kd_res_free(res);
    return true;
  }

  std::vector<PtsDefault*> pts;
  float                    z_med = 0.0;
  while (!kd_res_end(res)) {
    PtsDefault* pt = reinterpret_cast<PtsDefault*>(kd_res_item_data(res));
    pts.push_back(pt);
    kd_res_next(res);
  }
  kd_res_free(res);

  std::sort(pts.begin(), pts.end(), [](PtsDefault* a, PtsDefault* b) { return a->z < b->z; });
  z_med = pts[pts.size() / 2]->z;

  int total = pts.size();
  int cnt   = 0;
  for (auto& pt : pts) {
    if (fabs(pt->z - z_med) > param_.height_threshold) {
      cnt++;
    }
  }
  float ratio = static_cast<float>(cnt) / total;
  if (ratio > threshold) {
    return true;
  }
  return false;
}

bool TRG::isFrontier(Eigen::Vector2f& pos) {
  trgStruct&      global_graph = *trgMap_["global"];
  trgStruct&      local_graph  = *trgMap_["local"];
  Eigen::Vector2f dir          = pos - local_graph.root_pos;
  dir.normalize();

  Eigen::Vector2f check = pos + 2 * param_.robot_size * dir;

  kdres* res2 = kd_nearest_range2(global_graph.node_tree, check.x(), check.y(), param_.robot_size);
  if (kd_res_size(res2) > 0) {
    kd_res_free(res2);
    return false;
  }
  kd_res_free(res2);

  kdres* res1 =
      kd_nearest_range2(local_graph.map_tree, check.x(), check.y(), 0.5 * param_.robot_size);
  if (kd_res_size(res1) == 0) {
    kd_res_free(res1);
    return true;
  }
  kd_res_free(res1);
  return false;
}

std::unordered_map<int, TRG::Node*> TRG::getGraph(std::string type = "global") {
  trgStruct& graph = *trgMap_[type];
  return graph.nodes;
}

std::unordered_map<int, TRG::Node*> TRG::getGraphCopy(std::string type = "global") {
  std::lock_guard<std::mutex>         lock(mtx.graph);
  std::unordered_map<int, TRG::Node*> nodes_copy;
  trgStruct&                          graph = *trgMap_[type];
  for (auto& node : graph.nodes) {
    Eigen::Vector2f pos = node.second->pos_.head(2);
    Node* node_copy = new Node(node.second->id_, pos, node.second->pos_.z(), node.second->state_);
    for (auto& edge : node.second->edges_) {
      Edge* edge_copy = new Edge(edge->dst_id_, edge->weight_, edge->dist_);
      node_copy->edges_.push_back(edge_copy);
    }
    nodes_copy[node.first] = node_copy;
  }
  return nodes_copy;
}

void TRG::lockGraph() { mtx.graph.lock(); }

void TRG::unlockGraph() { mtx.graph.unlock(); }

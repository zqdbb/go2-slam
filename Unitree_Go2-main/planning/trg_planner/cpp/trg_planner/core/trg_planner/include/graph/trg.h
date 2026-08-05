/**
 * Copyright 2025, Korea Advanced Institute of Science and Technology
 * Massachusetts Institute of Technology,
 * Daejeon, 34051
 * All Rights Reserved
 * Authors: Dongkyu Lee, et al.
 * See LICENSE for the license information
 */
#ifndef CPP_TRG_PLANNER_CORE_TRG_PLANNER_INCLUDE_GRAPH_TRG_H_
#define CPP_TRG_PLANNER_CORE_TRG_PLANNER_INCLUDE_GRAPH_TRG_H_

#include "trg_planner/include/kdtree/kdtree.h"
#include "trg_planner/include/utils/common.h"

#define EPS 1e-6
const Eigen::Vector3f gravity(0, 0, -1);

class TRG {
 public:
  struct Edge {
    Edge(int dst_id, float weight, float dist) : dst_id_(dst_id), weight_(weight), dist_(dist) {}
    int   dst_id_;
    float weight_;
    float dist_;
  };

  enum struct NodeState {
    Valid    = 0,
    Invalid  = -1,
    Frontier = 1,
  };

  struct Node {
    Node(int id, Eigen::Vector2f& pos2d, float z, NodeState state)
        : id_(id), pos_(Eigen::Vector3f(pos2d.x(), pos2d.y(), z)), state_(state) {}
    int                id_;
    Eigen::Vector3f    pos_;
    NodeState          state_;
    std::vector<Edge*> edges_;
  };

  struct OptimizeNode {
    OptimizeNode(int i, float f, float g) : id_(i), f_(f), g_(g) {}
    int           id_;
    OptimizeNode* parent_;
    float         f_;
    float         g_;
  };

 public:
  TRG(bool  isVerbose,
      float expand_dist,
      float robot_size,
      int   sample_num,
      float height_threshold,
      float collision_threshold,
      float update_collision_threshold,
      float safety_factor,
      float goal_tolerance);
  virtual ~TRG() = default;

  void initGraph(bool isPreMap, Eigen::Vector3f start3d);

  void loadPrebuiltGraph();

  void setGlobalMap(PointCloudPtr& map);
  void setLocalMap(Eigen::Vector2f start2d, PointCloudPtr& map);
  void setLocalGraph(bool useMutex);

  bool addNode(int node_id, Eigen::Vector2f& node_pos, NodeState state, std::string type);
  void wireEdge(Node* node1, Node* node2, std::string type);

  void expandGraph(int node_id, std::string type);
  void cleanGraph(bool updateLocal);
  void updateGraph();

  void setGoal(Eigen::Vector3f& goal);
  bool checkReadched(Eigen::Vector2f& pos2d);
  bool checkReplan(Eigen::Vector2f& pos2d, std::vector<Eigen::Vector3f>& path);

  bool planSafePath(Eigen::Vector2f&              start2d,
                    Eigen::Vector3f&              goal_pose,
                    std::vector<Eigen::Vector3f>& out_path,
                    float&                        direct_dist,
                    float&                        path_length,
                    float&                        avg_risk);
  void refinePath(std::vector<Eigen::Vector3f>& in_path, std::vector<Eigen::Vector3f>& out_path);

  void resetGraph(std::string type);
  void resetMap(std::string type);

  bool isCollision(Eigen::Vector2f& pos2d, std::string type, float threshold);
  bool isFrontier(Eigen::Vector2f& pos2d);

  std::unordered_map<int, Node*> getGraph(std::string type);
  std::unordered_map<int, Node*> getGraphCopy(std::string type);
  void                           lockGraph();
  void                           unlockGraph();

 protected:
  struct trgStruct {
    explicit trgStruct(std::string type) : type(type), node_id(0) {}
    std::string type;

    std::unordered_map<int, Node*> nodes;
    kdtree*                        node_tree = kd_create(2);
    int                            node_id;
    Eigen::Vector2f                root_pos = Eigen::Vector2f::Zero();

    kdtree*       map_tree  = kd_create(2);
    PointCloudPtr cloud_map = nullptr;
  };
  trgStruct prebuilt_trg_ = trgStruct("prebuilt");
  trgStruct global_trg_   = trgStruct("global");
  trgStruct local_trg_    = trgStruct("local");

  std::unordered_map<std::string, trgStruct*> trgMap_ = {{"prebuilt", &prebuilt_trg_},
                                                         {"global", &global_trg_},
                                                         {"local", &local_trg_}};

  struct goalStruct {
    Eigen::Vector3f pose3d;
    Eigen::Vector2f pose2d;
    Node*           node    = nullptr;
    bool            isKnown = false;
  } goal_;

  std::random_device                    rd_;
  std::mt19937                          gen_;
  std::uniform_real_distribution<float> distr_;

  struct Param {
    bool  isVerbose                  = true;
    float expand_dist                = 0.5;
    float robot_size                 = 0.5;
    int   sample_num                 = 20;
    float height_threshold           = 0.5;
    float collision_threshold        = 0.5;
    float update_collision_threshold = 0.5;
    float safety_factor              = 3.0;
    float goal_tolerance             = 0.2;
  } param_;

  struct Mutex {
    std::mutex graph;
  } mtx;
};

#endif  // CPP_TRG_PLANNER_CORE_TRG_PLANNER_INCLUDE_GRAPH_TRG_H_

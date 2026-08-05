#ifndef PIPELINES_ROS2_INCLUDE_ROS2_NODE_H_
#define PIPELINES_ROS2_INCLUDE_ROS2_NODE_H_

#include "trg_planner/include/planner/trg_planner.h"
#include "trg_planner/include/utils/common.h"

#include "ros2_utils.hpp"

class ROS2Node : public TRGPlanner {
 public:
  explicit ROS2Node(const rclcpp::Node::SharedPtr &node);
  virtual ~ROS2Node();

  void getParams(const rclcpp::Node::SharedPtr &node);
  void cbPose(const std::shared_ptr<const ROS2Types::Pose> &msg);
  void cbOdom(const std::shared_ptr<const ROS2Types::Odom> &msg);
  void cbCloud(const std::shared_ptr<const ROS2Types::PointCloud> &msg);
  void cbGoal(const std::shared_ptr<const ROS2Types::Pose> &msg);

  void publishTimer();
  void debugTimer();

 protected:
  //// ROS2 Node
  rclcpp::Node::SharedPtr n_;
  //// ROS2 Subscribers
  struct Subscribers {
    rclcpp::Subscription<ROS2Types::Pose>::SharedPtr       ego_pose_;
    rclcpp::Subscription<ROS2Types::Odom>::SharedPtr       ego_odom_;
    rclcpp::Subscription<ROS2Types::PointCloud>::SharedPtr obs_cloud_;
    rclcpp::Subscription<ROS2Types::Pose>::SharedPtr       goal_;
  } sub;
  //// ROS2 Publishers
  struct Publishers {
    rclcpp::Publisher<ROS2Types::PointCloud>::SharedPtr pre_map_;
    rclcpp::Publisher<ROS2Types::PointCloud>::SharedPtr goal_;
    rclcpp::Publisher<ROS2Types::Path>::SharedPtr       path_;
  } pub;
  struct Debug {
    rclcpp::Publisher<ROS2Types::MarkerArray>::SharedPtr global_trg_;
    rclcpp::Publisher<ROS2Types::MarkerArray>::SharedPtr local_trg_;
    rclcpp::Publisher<ROS2Types::PointCloud>::SharedPtr  obs_map_;
    rclcpp::Publisher<ROS2Types::FloatArray>::SharedPtr  path_info_;
  } debug;
  struct QoS {
    rclcpp::QoS for_reli = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    rclcpp::QoS for_viz  = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  } qos;

  //// Parameters
  struct Parameters {
    bool        isDebug;
    std::string frame_id;
    float       publish_rate;
    float       debug_rate;
  } param_;

  std::unordered_map<std::string, std::string> topics_;

  //// Threads
  struct Threads {
    std::thread                            publish;
    std::thread                            debug;
    std::unordered_map<std::string, float> hz;
  } thd;

  //// TF
  struct TFCache {
    std::shared_ptr<tf2_ros::Buffer>            tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    geometry_msgs::msg::TransformStamped        tfStamped;
    bool                                        isTFCached = false;
  } tf_cache;

  void vizGraph(std::string type, rclcpp::Publisher<ROS2Types::MarkerArray>::SharedPtr pub);
};

#endif  // PIPELINES_ROS2_INCLUDE_ROS2_NODE_H_

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>                 
#include <std_msgs/msg/float64.hpp>              
#include <std_msgs/msg/bool.hpp>                 
#include <std_msgs/msg/float64_multi_array.hpp>  
#include <atomic>
#include <mutex>
#include <string>
#include <vector>
#include <optional>

namespace elevator_bt {

class ElevatorClient {
public:
  explicit ElevatorClient(const rclcpp::Node::SharedPtr& node);

  void setNamespace(const std::string& ns);

  void cmdFloor(int floor_idx);     // /<ns>/cmd_floor : std_msgs/Int32
  void cmdZ(double z_m);            // /<ns>/cmd_z     : std_msgs/Float64
  void setDoorOpen(bool open);      // /<ns>/door_open : std_msgs/Bool

  // 상태 조회
  double cabinZ() const;                                
  std::vector<double> doorPos() const;                  
  double doorOpenExtent() const;                        
  double doorA() const;                                 
  double doorB() const;                                 

private:
  rclcpp::Node::SharedPtr node_;
  std::string ns_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr            pub_cmd_floor_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr          pub_cmd_z_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr             pub_door_open_;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr             sub_cabin_z_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr   sub_door_pos_;

  // Latest states
  std::atomic<double> cabin_z_{std::numeric_limits<double>::quiet_NaN()};
  std::atomic<double> door_a_{std::numeric_limits<double>::quiet_NaN()};
  std::atomic<double> door_b_{std::numeric_limits<double>::quiet_NaN()};

  mutable std::mutex mtx_door_vec_;
  std::vector<double> door_vec_; 
};

} // namespace elevator_bt

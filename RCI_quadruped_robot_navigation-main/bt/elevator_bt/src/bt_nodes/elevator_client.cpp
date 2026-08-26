#include "elevator_bt/bt_nodes/elevator_client.hpp"
#include <limits>
#include <cmath>

using std_msgs::msg::Int32;
using std_msgs::msg::Float64;
using std_msgs::msg::Bool;
using std_msgs::msg::Float64MultiArray;

namespace elevator_bt {

ElevatorClient::ElevatorClient(const rclcpp::Node::SharedPtr& node)
: node_(node) {}

void ElevatorClient::setNamespace(const std::string& ns)
{
  ns_ = ns;

  // pubs
  pub_cmd_floor_ = node_->create_publisher<Int32>(ns_ + "/cmd_floor", 10);
  pub_cmd_z_     = node_->create_publisher<Float64>(ns_ + "/cmd_z", 10);
  pub_door_open_ = node_->create_publisher<Bool>(ns_ + "/door_open", 10);

  // subs
  sub_cabin_z_ = node_->create_subscription<Float64>(
      ns_ + "/cabin_z", rclcpp::QoS(10),
      [this](const Float64::SharedPtr msg){
        cabin_z_.store(msg->data, std::memory_order_relaxed);
      });

  sub_door_pos_ = node_->create_subscription<Float64MultiArray>(
      ns_ + "/door_pos", rclcpp::QoS(10),
      [this](const Float64MultiArray::SharedPtr msg){
        double a = std::numeric_limits<double>::quiet_NaN();
        double b = std::numeric_limits<double>::quiet_NaN();
        if (!msg->data.empty()) {
          a = msg->data.size() > 0 ? msg->data[0] : a;
          b = msg->data.size() > 1 ? msg->data[1] : b;
        }
        door_a_.store(a, std::memory_order_relaxed);
        door_b_.store(b, std::memory_order_relaxed);
        {
          std::lock_guard<std::mutex> lk(mtx_door_vec_);
          door_vec_ = msg->data;
        }
      });
}

void ElevatorClient::cmdFloor(int floor_idx)
{
  if (!pub_cmd_floor_) return;
  Int32 m; m.data = floor_idx;
  pub_cmd_floor_->publish(m);
}

void ElevatorClient::cmdZ(double z_m)
{
  if (!pub_cmd_z_) return;
  Float64 m; m.data = z_m;
  pub_cmd_z_->publish(m);
}

void ElevatorClient::setDoorOpen(bool open)
{
  if (!pub_door_open_) return;
  Bool m; m.data = open;
  pub_door_open_->publish(m);
}

double ElevatorClient::cabinZ() const
{
  return cabin_z_.load(std::memory_order_relaxed);
}

std::vector<double> ElevatorClient::doorPos() const
{
  std::lock_guard<std::mutex> lk(mtx_door_vec_);
  return door_vec_;
}

double ElevatorClient::doorOpenExtent() const
{
  double a = door_a_.load(std::memory_order_relaxed);
  double b = door_b_.load(std::memory_order_relaxed);
  if (std::isnan(a) && std::isnan(b)) return std::numeric_limits<double>::quiet_NaN();
  if (std::isnan(a)) return std::abs(b);
  if (std::isnan(b)) return std::abs(a);
  return 0.5 * (std::abs(a) + std::abs(b));
}

double ElevatorClient::doorA() const { return door_a_.load(std::memory_order_relaxed); }
double ElevatorClient::doorB() const { return door_b_.load(std::memory_order_relaxed); }

} // namespace elevator_bt

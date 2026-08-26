/*
 * Copyright (c) 2024-2025 Ziqi Fan (Modified by Sanghyun Kim)
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_SIM_HPP
#define RL_SIM_HPP

// #define PLOT
// #define CSV_LOGGER

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "loop.hpp"
#include "fsm.hpp"

#include <csignal>
#include <vector>
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "robot_msgs/msg/robot_command.hpp"
#include "robot_msgs/msg/robot_state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rl_sar/action/robot_activation.hpp"
#include "rl_sar/action/set_locomotion_mode.hpp"
#include "rl_sar/action/set_navigation_mode.hpp"

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

class RL_Sim : public RL, public rclcpp::Node
{
public:
    RL_Sim();
    ~RL_Sim();

private:
    // rl functions
    torch::Tensor Forward() override;
    void GetState(RobotState<double> *state) override;
    void SetCommand(const RobotCommand<double> *command) override;
    void RunModel();
    void RobotControl();

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;

    // plot
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<double>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();

    // ros interface
    std::string ros_namespace;
    sensor_msgs::msg::Imu gazebo_imu;
    geometry_msgs::msg::Twist cmd_vel;
    sensor_msgs::msg::Joy joy_msg;
    robot_msgs::msg::RobotCommand robot_command_publisher_msg;
    robot_msgs::msg::RobotState robot_state_subscriber_msg;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gazebo_imu_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr gazebo_pause_physics_client;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr gazebo_unpause_physics_client;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr gazebo_reset_world_client;
    rclcpp::Publisher<robot_msgs::msg::RobotCommand>::SharedPtr robot_command_publisher;
    rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr robot_state_subscriber;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr param_client;
    void GazeboImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void RobotStateCallback(const robot_msgs::msg::RobotState::SharedPtr msg);
    void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    // Action Server
    using RobotActivation = rl_sar::action::RobotActivation;
    using GoalHandleRobotActivation = rclcpp_action::ServerGoalHandle<RobotActivation>;
    rclcpp_action::Server<RobotActivation>::SharedPtr activation_server_;
    rclcpp_action::GoalResponse handle_goal_activation(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const RobotActivation::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel_activation(const std::shared_ptr<GoalHandleRobotActivation> goal_handle);
    void handle_accepted_activation(const std::shared_ptr<GoalHandleRobotActivation> goal_handle);
    void execute_activation(const std::shared_ptr<GoalHandleRobotActivation> goal_handle);

    using SetLocomotionMode = rl_sar::action::SetLocomotionMode;
    using GoalHandleSetLocomotionMode = rclcpp_action::ServerGoalHandle<SetLocomotionMode>;
    rclcpp_action::Server<SetLocomotionMode>::SharedPtr locomotion_mode_server_;
    rclcpp_action::GoalResponse handle_goal_locomotion_mode(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const SetLocomotionMode::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel_locomotion_mode(const std::shared_ptr<GoalHandleSetLocomotionMode> goal_handle);
    void handle_accepted_locomotion_mode(const std::shared_ptr<GoalHandleSetLocomotionMode> goal_handle);
    void execute_locomotion_mode(const std::shared_ptr<GoalHandleSetLocomotionMode> goal_handle);

    using SetNavigationMode = rl_sar::action::SetNavigationMode;
    using GoalHandleSetNavigationMode = rclcpp_action::ServerGoalHandle<SetNavigationMode>;
    rclcpp_action::Server<SetNavigationMode>::SharedPtr navigation_mode_server_;
    rclcpp_action::GoalResponse handle_goal_navigation_mode(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const SetNavigationMode::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel_navigation_mode(const std::shared_ptr<GoalHandleSetNavigationMode> goal_handle);
    void handle_accepted_navigation_mode(const std::shared_ptr<GoalHandleSetNavigationMode> goal_handle);
    void execute_navigation_mode(const std::shared_ptr<GoalHandleSetNavigationMode> goal_handle);

    // others
    std::string gazebo_model_name;
    int motiontime = 0;
    std::map<std::string, double> joint_positions;
    std::map<std::string, double> joint_velocities;
    std::map<std::string, double> joint_efforts;
    void StartJointController(const std::string& ros_namespace, const std::vector<std::string>& names);
};

#endif // RL_SIM_HPP

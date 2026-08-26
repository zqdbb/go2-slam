/*
 * Copyright (c) 2024-2025 Ziqi Fan (Modified by Sanghyun Kim)
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sim.hpp"

RL_Sim::RL_Sim() : rclcpp::Node("rl_sim_node")
{
    this->ang_vel_type = "ang_vel_body";
    this->ros_namespace = this->get_namespace();
    // get params from param_node
    param_client = this->create_client<rcl_interfaces::srv::GetParameters>("/param_node/get_parameters");
    while (!param_client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            std::cout << LOGGER::ERROR << "Interrupted while waiting for param_node service. Exiting." << std::endl;
            return;
        }
        std::cout << LOGGER::WARNING << "Waiting for param_node service to be available..." << std::endl;
    }
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {"robot_name", "gazebo_model_name"};
    // Use a timeout for the future
    auto future = param_client->async_send_request(request);
    auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(5));
    if (status == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto result = future.get();
        if (result->values.size() < 2)
        {
            std::cout << LOGGER::ERROR << "Failed to get all parameters from param_node" << std::endl;
        }
        else
        {
            this->robot_name = result->values[0].string_value;
            this->gazebo_model_name = result->values[1].string_value;
            std::cout << LOGGER::INFO << "Get param robot_name: " << this->robot_name << std::endl;
            std::cout << LOGGER::INFO << "Get param gazebo_model_name: " << this->gazebo_model_name << std::endl;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "Failed to call param_node service" << std::endl;
    }

    // read params from yaml
    this->ReadYamlBase(this->robot_name);

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // init torch
    torch::autograd::GradMode::set_enabled(false);
    torch::set_num_threads(4);

    // init robot
    this->robot_command_publisher_msg.motor_command.resize(this->params.num_of_dofs);
    this->robot_state_subscriber_msg.motor_state.resize(this->params.num_of_dofs);
    this->InitOutputs();
    this->InitControl();

    this->StartJointController(this->ros_namespace, this->params.joint_names);
    // publisher
    this->robot_command_publisher = this->create_publisher<robot_msgs::msg::RobotCommand>(
        this->ros_namespace + "robot_joint_controller/command", rclcpp::SystemDefaultsQoS());

    // subscriber
    this->cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        { this->CmdvelCallback(msg); });
    this->joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Joy::SharedPtr msg)
        { this->JoyCallback(msg); });
    this->gazebo_imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SystemDefaultsQoS(), [this](const sensor_msgs::msg::Imu::SharedPtr msg)
        { this->GazeboImuCallback(msg); });
    this->robot_state_subscriber = this->create_subscription<robot_msgs::msg::RobotState>(
        this->ros_namespace + "robot_joint_controller/state", rclcpp::SystemDefaultsQoS(),
        [this](const robot_msgs::msg::RobotState::SharedPtr msg)
        { this->RobotStateCallback(msg); });

    // service
    this->gazebo_pause_physics_client = this->create_client<std_srvs::srv::Empty>("/pause_physics");
    this->gazebo_unpause_physics_client = this->create_client<std_srvs::srv::Empty>("/unpause_physics");
    this->gazebo_reset_world_client = this->create_client<std_srvs::srv::Empty>("/reset_world");

    auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result = this->gazebo_reset_world_client->async_send_request(empty_request);

    // loop
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.dt, std::bind(&RL_Sim::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.dt * this->params.decimation, std::bind(&RL_Sim::RunModel, this));
    this->loop_control->start();
    this->loop_rl->start();

    // keyboard
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Sim::KeyboardInterface, this));
    this->loop_keyboard->start();

    // here
    // action server
    using namespace std::placeholders;
    this->activation_server_ = rclcpp_action::create_server<RobotActivation>(
        this,
        "robot_activation",
        std::bind(&RL_Sim::handle_goal_activation, this, _1, _2),
        std::bind(&RL_Sim::handle_cancel_activation, this, _1),
        std::bind(&RL_Sim::handle_accepted_activation, this, _1));

    this->locomotion_mode_server_ = rclcpp_action::create_server<SetLocomotionMode>(
        this,
        "set_locomotion_mode",
        std::bind(&RL_Sim::handle_goal_locomotion_mode, this, _1, _2),
        std::bind(&RL_Sim::handle_cancel_locomotion_mode, this, _1),
        std::bind(&RL_Sim::handle_accepted_locomotion_mode, this, _1));

    this->navigation_mode_server_ = rclcpp_action::create_server<SetNavigationMode>(
        this,
        "set_navigation_mode",
        std::bind(&RL_Sim::handle_goal_navigation_mode, this, _1, _2),
        std::bind(&RL_Sim::handle_cancel_navigation_mode, this, _1),
        std::bind(&RL_Sim::handle_accepted_navigation_mode, this, _1));
    // here
    std::cout << LOGGER::INFO << "set navigation" << std::endl;

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.num_of_dofs);
    this->plot_target_joint_pos.resize(this->params.num_of_dofs);
    for (auto &vector : this->plot_real_joint_pos)
    {
        vector = std::vector<double>(this->plot_size, 0);
    }
    for (auto &vector : this->plot_target_joint_pos)
    {
        vector = std::vector<double>(this->plot_size, 0);
    }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.001, std::bind(&RL_Sim::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif

    std::cout << LOGGER::INFO << "RL_Sim start" << std::endl;
}

RL_Sim::~RL_Sim()
{
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Sim exit" << std::endl;
}

// here
// RobotActivation Action Server
rclcpp_action::GoalResponse RL_Sim::handle_goal_activation(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const RobotActivation::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request for RobotActivation");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RL_Sim::handle_cancel_activation(
    const std::shared_ptr<GoalHandleRobotActivation> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal for RobotActivation");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void RL_Sim::handle_accepted_activation(
    const std::shared_ptr<GoalHandleRobotActivation> goal_handle)
{
    using namespace std::placeholders;
    std::thread{std::bind(&RL_Sim::execute_activation, this, _1), goal_handle}.detach();
}

void RL_Sim::execute_activation(
    const std::shared_ptr<GoalHandleRobotActivation> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal for RobotActivation");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<RobotActivation::Result>();
    auto feedback = std::make_shared<RobotActivation::Feedback>();

    if (goal->activate)
    {
        RCLCPP_INFO(this->get_logger(), "Action Server: Robot Activation request received.");
        RCLCPP_INFO(this->get_logger(), "Activating robot...");
        feedback->status = "Activating robot";
        goal_handle->publish_feedback(feedback);
        this->control.SetKeyboard(Input::Keyboard::Num0);

        rclcpp::sleep_for(std::chrono::milliseconds(500));

        if (!goal_handle->is_active() || goal_handle->is_canceling())
        {
            result->success = false;
            result->message = "Goal canceled";
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Setting default walking mode...");
        feedback->status = "Setting default walking mode";
        goal_handle->publish_feedback(feedback);
        this->control.SetKeyboard(Input::Keyboard::Num1);

        rclcpp::sleep_for(std::chrono::milliseconds(100));

        result->success = true;
        result->message = "Robot activated and in default walking mode.";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Activation goal succeeded");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Action Server: Robot Deactivation request received.");
        RCLCPP_INFO(this->get_logger(), "Deactivating robot...");
        feedback->status = "Deactivating robot";
        goal_handle->publish_feedback(feedback);
        this->control.SetKeyboard(Input::Keyboard::Num9);

        rclcpp::sleep_for(std::chrono::milliseconds(100));

        result->success = true;
        result->message = "Robot deactivated.";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Deactivation goal succeeded");
    }
}

// SetLocomotionMode Action Server
rclcpp_action::GoalResponse RL_Sim::handle_goal_locomotion_mode(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const SetLocomotionMode::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request for SetLocomotionMode with mode %d", goal->mode);
    (void)uuid;
    if (goal->mode < 1 || goal->mode > 8)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid locomotion mode. Must be between 1 and 8.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RL_Sim::handle_cancel_locomotion_mode(
    const std::shared_ptr<GoalHandleSetLocomotionMode> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal for SetLocomotionMode");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void RL_Sim::handle_accepted_locomotion_mode(
    const std::shared_ptr<GoalHandleSetLocomotionMode> goal_handle)
{
    using namespace std::placeholders;
    std::thread{std::bind(&RL_Sim::execute_locomotion_mode, this, _1), goal_handle}.detach();
}

void RL_Sim::execute_locomotion_mode(
    const std::shared_ptr<GoalHandleSetLocomotionMode> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal for SetLocomotionMode");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<SetLocomotionMode::Result>();

    RCLCPP_INFO(this->get_logger(), "Action Server: Set Locomotion Mode to %d request received.", goal->mode);
    Input::Keyboard key = static_cast<Input::Keyboard>(static_cast<int>(Input::Keyboard::Num1) + goal->mode - 1);
    this->control.SetKeyboard(key);

    rclcpp::sleep_for(std::chrono::milliseconds(100));

    result->success = true;
    result->message = "Set locomotion mode to " + std::to_string(goal->mode);
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "SetLocomotionMode goal succeeded");
}

// SetNavigationMode Action Server
rclcpp_action::GoalResponse RL_Sim::handle_goal_navigation_mode(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const SetNavigationMode::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request for SetNavigationMode");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RL_Sim::handle_cancel_navigation_mode(
    const std::shared_ptr<GoalHandleSetNavigationMode> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal for SetNavigationMode");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void RL_Sim::handle_accepted_navigation_mode(
    const std::shared_ptr<GoalHandleSetNavigationMode> goal_handle)
{
    using namespace std::placeholders;
    std::thread{std::bind(&RL_Sim::execute_navigation_mode, this, _1), goal_handle}.detach();
}

void RL_Sim::execute_navigation_mode(
    const std::shared_ptr<GoalHandleSetNavigationMode> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal for SetNavigationMode");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<SetNavigationMode::Result>();

    RCLCPP_INFO(this->get_logger(), "Action Server: Set Navigation Mode to %s request received.", goal->enable ? "ON" : "OFF");
    if (this->control.navigation_mode != goal->enable)
    {
        this->control.SetKeyboard(Input::Keyboard::N);
        result->message = "Navigation mode toggled to " + std::string(goal->enable ? "ON" : "OFF");
    }
    else
    {
        result->message = "Navigation mode is already " + std::string(goal->enable ? "ON" : "OFF");
    }

    rclcpp::sleep_for(std::chrono::milliseconds(100));

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "SetNavigationMode goal succeeded");
}
// here

void RL_Sim::StartJointController(const std::string &ros_namespace, const std::vector<std::string> &names)
{
    const char *ros_distro = std::getenv("ROS_DISTRO");
    std::string spawner = (ros_distro && std::string(ros_distro) == "foxy") ? "spawner.py" : "spawner";

    std::filesystem::path tmp_path = std::filesystem::temp_directory_path() / "robot_joint_controller_params.yaml";
    {
        std::ofstream tmp_file(tmp_path);
        if (!tmp_file)
        {
            throw std::runtime_error("Failed to create temporary parameter file");
        }

        tmp_file << "/robot_joint_controller:\n";
        tmp_file << "    ros__parameters:\n";
        tmp_file << "        joints:\n";
        for (const auto &name : names)
        {
            tmp_file << "            - " << name << "\n";
        }
    }

    pid_t pid = fork();
    if (pid == 0)
    {
        std::string cmd = "ros2 run controller_manager " + spawner + " robot_joint_controller ";
        cmd += "-p " + tmp_path.string() + " ";
        // cmd += " > /dev/null 2>&1";  // Comment this line to see the output
        execlp("sh", "sh", "-c", cmd.c_str(), nullptr);
        exit(1);
    }
    else if (pid > 0)
    {
        int status;
        waitpid(pid, &status, 0);

        if (WIFEXITED(status) && WEXITSTATUS(status) != 0)
        {
            throw std::runtime_error("Failed to start joint controller");
        }

        std::filesystem::remove(tmp_path);
    }
    else
    {
        throw std::runtime_error("fork() failed");
    }
}

void RL_Sim::GetState(RobotState<double> *state)
{
    const auto &orientation = this->gazebo_imu.orientation;
    const auto &angular_velocity = this->gazebo_imu.angular_velocity;

    state->imu.quaternion[0] = orientation.w;
    state->imu.quaternion[1] = orientation.x;
    state->imu.quaternion[2] = orientation.y;
    state->imu.quaternion[3] = orientation.z;

    state->imu.gyroscope[0] = angular_velocity.x;
    state->imu.gyroscope[1] = angular_velocity.y;
    state->imu.gyroscope[2] = angular_velocity.z;

    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        state->motor_state.q[i] = this->robot_state_subscriber_msg.motor_state[this->params.joint_mapping[i]].q;
        state->motor_state.dq[i] = this->robot_state_subscriber_msg.motor_state[this->params.joint_mapping[i]].dq;
        state->motor_state.tau_est[i] = this->robot_state_subscriber_msg.motor_state[this->params.joint_mapping[i]].tau_est;
    }
}

void RL_Sim::SetCommand(const RobotCommand<double> *command)
{
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->robot_command_publisher_msg.motor_command[this->params.joint_mapping[i]].q = command->motor_command.q[i];
        this->robot_command_publisher_msg.motor_command[this->params.joint_mapping[i]].dq = command->motor_command.dq[i];
        this->robot_command_publisher_msg.motor_command[this->params.joint_mapping[i]].kp = command->motor_command.kp[i];
        this->robot_command_publisher_msg.motor_command[this->params.joint_mapping[i]].kd = command->motor_command.kd[i];
        this->robot_command_publisher_msg.motor_command[this->params.joint_mapping[i]].tau = command->motor_command.tau[i];
    }

    this->robot_command_publisher->publish(this->robot_command_publisher_msg);
}

void RL_Sim::RobotControl()
{
    if (this->control.current_keyboard == Input::Keyboard::R || this->control.current_gamepad == Input::Gamepad::RB_Y)
    {
        auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = this->gazebo_reset_world_client->async_send_request(empty_request);
        this->control.current_keyboard = this->control.last_keyboard;
    }
    if (this->control.current_keyboard == Input::Keyboard::P || this->control.current_gamepad == Input::Gamepad::RB_X)
    {
        if (simulation_running)
        {
            auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = this->gazebo_pause_physics_client->async_send_request(empty_request);
            std::cout << std::endl
                      << LOGGER::INFO << "Simulation Stop" << std::endl;
        }
        else
        {
            auto empty_request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = this->gazebo_unpause_physics_client->async_send_request(empty_request);
            std::cout << std::endl
                      << LOGGER::INFO << "Simulation Start" << std::endl;
        }
        simulation_running = !simulation_running;
        this->control.current_keyboard = this->control.last_keyboard;
    }

    if (simulation_running)
    {
        this->motiontime++;

        if (this->control.current_keyboard == Input::Keyboard::W)
        {
            this->control.x += 0.1;
            this->control.current_keyboard = this->control.last_keyboard;
        }
        if (this->control.current_keyboard == Input::Keyboard::S)
        {
            this->control.x -= 0.1;
            this->control.current_keyboard = this->control.last_keyboard;
        }
        if (this->control.current_keyboard == Input::Keyboard::A)
        {
            this->control.y += 0.1;
            this->control.current_keyboard = this->control.last_keyboard;
        }
        if (this->control.current_keyboard == Input::Keyboard::D)
        {
            this->control.y -= 0.1;
            this->control.current_keyboard = this->control.last_keyboard;
        }
        if (this->control.current_keyboard == Input::Keyboard::Q)
        {
            this->control.yaw += 0.1;
            this->control.current_keyboard = this->control.last_keyboard;
        }
        if (this->control.current_keyboard == Input::Keyboard::E)
        {
            this->control.yaw -= 0.1;
            this->control.current_keyboard = this->control.last_keyboard;
        }
        if (this->control.current_keyboard == Input::Keyboard::Space)
        {
            this->control.x = 0;
            this->control.y = 0;
            this->control.yaw = 0;
            this->control.current_keyboard = this->control.last_keyboard;
            std::cout << std::endl
                      << LOGGER::INFO << "Space " << std::endl;
        }
        if (this->control.current_keyboard == Input::Keyboard::N || this->control.current_gamepad == Input::Gamepad::X)
        {
            this->control.navigation_mode = !this->control.navigation_mode;
            std::cout << std::endl
                      << LOGGER::INFO << "Navigation mode: " << (this->control.navigation_mode ? "ON" : "OFF") << std::endl;
            this->control.current_keyboard = this->control.last_keyboard;
        }

        this->GetState(&this->robot_state);
        this->StateController(&this->robot_state, &this->robot_command);
        this->SetCommand(&this->robot_command);
    }
}

void RL_Sim::GazeboImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    this->gazebo_imu = *msg;
}

void RL_Sim::CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->cmd_vel = *msg;
}

void RL_Sim::JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    this->joy_msg = *msg;

    // joystick control
    // Description of buttons and axes(F710):
    // |__ buttons[]: A=0, B=1, X=2, Y=3, LB=4, RB=5, back=6, start=7, power=8, stickL=9, stickR=10
    // |__ axes[]: Lx=0, Ly=1, Rx=3, Ry=4, LT=2, RT=5, DPadX=6, DPadY=7

    if (this->joy_msg.buttons[0])
        this->control.SetGamepad(Input::Gamepad::A);
    if (this->joy_msg.buttons[1])
        this->control.SetGamepad(Input::Gamepad::B);
    if (this->joy_msg.buttons[2])
        this->control.SetGamepad(Input::Gamepad::X);
    if (this->joy_msg.buttons[3])
        this->control.SetGamepad(Input::Gamepad::Y);
    if (this->joy_msg.buttons[4])
        this->control.SetGamepad(Input::Gamepad::LB);
    if (this->joy_msg.buttons[5])
        this->control.SetGamepad(Input::Gamepad::RB);
    if (this->joy_msg.buttons[9])
        this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->joy_msg.buttons[10])
        this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->joy_msg.axes[7] > 0)
        this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->joy_msg.axes[7] < 0)
        this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->joy_msg.axes[6] < 0)
        this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->joy_msg.axes[6] > 0)
        this->control.SetGamepad(Input::Gamepad::DPadRight);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[0])
        this->control.SetGamepad(Input::Gamepad::LB_A);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[1])
        this->control.SetGamepad(Input::Gamepad::LB_B);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[2])
        this->control.SetGamepad(Input::Gamepad::LB_X);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[3])
        this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[9])
        this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[10])
        this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[7] > 0)
        this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[7] < 0)
        this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[6] > 0)
        this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (this->joy_msg.buttons[4] && this->joy_msg.axes[6] < 0)
        this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[0])
        this->control.SetGamepad(Input::Gamepad::RB_A);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[1])
        this->control.SetGamepad(Input::Gamepad::RB_B);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[2])
        this->control.SetGamepad(Input::Gamepad::RB_X);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[3])
        this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[9])
        this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if (this->joy_msg.buttons[5] && this->joy_msg.buttons[10])
        this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[7] > 0)
        this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[7] < 0)
        this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[6] > 0)
        this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (this->joy_msg.buttons[5] && this->joy_msg.axes[6] < 0)
        this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (this->joy_msg.buttons[4] && this->joy_msg.buttons[5])
        this->control.SetGamepad(Input::Gamepad::LB_RB);

    this->control.x = this->joy_msg.axes[1] * 1.5;   // LY
    this->control.y = this->joy_msg.axes[0] * 1.5;   // LX
    this->control.yaw = this->joy_msg.axes[3] * 1.5; // RX
}

void RL_Sim::RobotStateCallback(const robot_msgs::msg::RobotState::SharedPtr msg)
{
    this->robot_state_subscriber_msg = *msg;
}

void RL_Sim::RunModel()
{
    if (this->rl_init_done && simulation_running)
    {
        this->episode_length_buf += 1;
        // this->obs.lin_vel = torch::tensor({{this->cmd_vel.linear.x, this->cmd_vel.linear.y, this->cmd_vel.linear.z}});
        this->obs.ang_vel = torch::tensor(this->robot_state.imu.gyroscope).unsqueeze(0);
        if (this->control.navigation_mode)
        {
            this->obs.commands = torch::tensor({{this->cmd_vel.linear.x, this->cmd_vel.linear.y, this->cmd_vel.angular.z}});
        }
        else
        {
            this->obs.commands = torch::tensor({{this->control.x, this->control.y, this->control.yaw}});
        }
        this->obs.base_quat = torch::tensor(this->robot_state.imu.quaternion).unsqueeze(0);
        this->obs.dof_pos = torch::tensor(this->robot_state.motor_state.q).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);
        this->obs.dof_vel = torch::tensor(this->robot_state.motor_state.dq).narrow(0, 0, this->params.num_of_dofs).unsqueeze(0);

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (this->output_dof_pos.defined() && this->output_dof_pos.numel() > 0)
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (this->output_dof_vel.defined() && this->output_dof_vel.numel() > 0)
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (this->output_dof_tau.defined() && this->output_dof_tau.numel() > 0)
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

        // this->TorqueProtect(this->output_dof_tau);

#ifdef CSV_LOGGER
        torch::Tensor tau_est = torch::zeros({1, this->params.num_of_dofs});
        for (int i = 0; i < this->params.num_of_dofs; ++i)
        {
            tau_est[0][i] = this->joint_efforts[this->params.joint_controller_names[i]];
        }
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

torch::Tensor RL_Sim::Forward()
{
    torch::autograd::GradMode::set_enabled(false);

    torch::Tensor clamped_obs = this->ComputeObservation();

    torch::Tensor actions;
    if (this->params.observations_history.size() != 0)
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.observations_history);
        actions = this->model.forward({this->history_obs}).toTensor();
    }
    else
    {
        actions = this->model.forward({clamped_obs}).toTensor();
    }

    if (this->params.clip_actions_upper.numel() != 0 && this->params.clip_actions_lower.numel() != 0)
    {
        return torch::clamp(actions, this->params.clip_actions_lower, this->params.clip_actions_upper);
    }
    else
    {
        return actions;
    }
}

void RL_Sim::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.num_of_dofs; ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(this->robot_state_subscriber_msg.motor_state[i].q);
        this->plot_target_joint_pos[i].push_back(this->robot_command_publisher_msg.motor_command[i].q);
        plt::subplot(this->params.num_of_dofs, 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.01);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RL_Sim>());
    rclcpp::shutdown();
    return 0;
}

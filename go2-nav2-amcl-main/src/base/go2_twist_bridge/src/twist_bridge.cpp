#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sport_model.hpp"
#include "nlohmann/json.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>

using namespace std::placeholders;

class TwistBridge : public rclcpp::Node
{
public:
    TwistBridge() : Node("twist_bridge_node_cpp")
    {   
        max_linear_ = declare_parameter<double>("max_linear_speed", 0.35);
        max_lateral_ = declare_parameter<double>("max_lateral_speed", 0.20);
        max_angular_ = declare_parameter<double>("max_angular_speed", 0.80);
        command_timeout_ = declare_parameter<double>("command_timeout", 0.50);
        last_command_ = now();
        RCLCPP_INFO(this->get_logger(), "TwistBridge创建，可以将geometry_msgs/msg/twist消息转换成unitree_api/msg/request消息!");
        //创建一个request发布对象
        request_pub_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        //创建一个twist订阅对象
        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&TwistBridge::twist_cb, this, _1));
        watchdog_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&TwistBridge::watchdog_cb, this));
    }

private:
    void twist_cb(const geometry_msgs::msg::Twist::SharedPtr twist)
    {
        //3-3.在回调函数中实现消息的转换以及发布
        unitree_api::msg::Request request;

        //转换
        //获取 twist 消息的线速度和角速度
        if (!std::isfinite(twist->linear.x) || !std::isfinite(twist->linear.y) ||
            !std::isfinite(twist->angular.z)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Ignoring non-finite cmd_vel");
            return;
        }
        double x = std::clamp(twist->linear.x, -max_linear_, max_linear_);
        double y = std::clamp(twist->linear.y, -max_lateral_, max_lateral_);
        double z = std::clamp(twist->angular.z, -max_angular_, max_angular_);
        last_command_ = now();
        watchdog_sent_ = false;
        //默认api_id为平衡站立
        auto api_id = ROBOT_SPORT_API_ID_BALANCESTAND;

        if(x != 0 || y != 0 || z != 0)
        {
            api_id = ROBOT_SPORT_API_ID_MOVE;

            nlohmann::json js;
            js["x"] = x;
            js["y"] = y;
            js["z"] = z;
            request.parameter = js.dump();
        }
        request.header.identity.api_id = api_id;
        request_pub_->publish(request);
    }

    void watchdog_cb()
    {
        if ((now() - last_command_).seconds() <= command_timeout_)
            return;
        if (watchdog_sent_)
            return;
        unitree_api::msg::Request request;
        request.header.identity.api_id = ROBOT_SPORT_API_ID_BALANCESTAND;
        request_pub_->publish(request);
        watchdog_sent_ = true;
    }

    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_;
    rclcpp::Time last_command_;
    double max_linear_{0.35}, max_lateral_{0.20}, max_angular_{0.80}, command_timeout_{0.50};
    bool watchdog_sent_{false};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistBridge>());
    rclcpp::shutdown();
    return 0;
}

//
// Created by spaaaaace on 2026/4/28.
//

#include "../include/servo_command_node/twist_publisher.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class TwistPublisher : public rclcpp::Node
{
public:
    TwistPublisher()
    : Node("twist_publisher")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),   // 50Hz
            std::bind(&TwistPublisher::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Twist publisher started.");
    }

private:
    void timer_callback()
    {
        auto msg = geometry_msgs::msg::TwistStamped();

        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";  //必须和 planning_frame 一致

        msg.twist.linear.x = 0.1;
        msg.twist.linear.y = 0.0;
        msg.twist.linear.z = 0.0;

        msg.twist.angular.x = 0.0;
        msg.twist.angular.y = 0.0;
        msg.twist.angular.z = 0.0;

        pub_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistPublisher>());
    rclcpp::shutdown();
    return 0;
}
//
// Created by spaaaaace on 2026/4/28.
//

#include "../include/servo_command_node/twist_publisher.h"

TwistPublisher::TwistPublisher() : Node("twist_publisher")
{

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&TwistPublisher::joyControllerCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/servo_node/delta_twist_cmds", 10);

    RCLCPP_INFO(this->get_logger(), "Twist publisher started.");
}

void TwistPublisher::publishTwistMsg(geometry_msgs::msg::TwistStamped msg)
{
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";

    pub_->publish(msg);
}

void TwistPublisher::joyControllerCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (msg->axes.size() < 6 || msg->buttons.size() < 8)
    {
        RCLCPP_WARN(this->get_logger(), "Joy message size not enough!");
        return;
    }
    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.twist.linear.x = msg->axes[0];  //左遥左右
    twist_msg.twist.linear.y = msg->axes[1];  //左遥上下
    twist_msg.twist.linear.z = ((msg->axes[4] - 1.0f) + (msg->axes[5] - 1.0f) * -1.0f) / 2.0f;   //左右扳机

    twist_msg.twist.angular.z = msg->axes[2]; //右遥左右
    twist_msg.twist.angular.x = msg->axes[3]; //右遥上下
    twist_msg.twist.angular.y = msg->buttons[6] + msg->buttons[7] * -1.0f;  //左右肩键

    publishTwistMsg(twist_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistPublisher>());
    rclcpp::shutdown();
    return 0;
}
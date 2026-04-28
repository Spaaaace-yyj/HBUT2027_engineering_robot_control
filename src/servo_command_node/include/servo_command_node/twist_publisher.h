//
// Created by spaaaaace on 2026/4/28.
//

#ifndef BUILD_TWIST_PUBLISHER_H
#define BUILD_TWIST_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include <functional>

class TwistPublisher : public rclcpp::Node
{
public:
    TwistPublisher();

    void joyControllerCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    void publishTwistMsg(geometry_msgs::msg::TwistStamped msg);

private:

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;

};

#endif //BUILD_TWIST_PUBLISHER_H
#ifndef HELLO_MOVEIT_HPP
#define HELLO_MOVEIT_HPP
#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include <moveit_visual_tools/moveit_visual_tools.h>

class HelloMoveIt : public rclcpp::Node{
public:
    HelloMoveIt();

    void runDemo();       // 执行流程

    void initMoveGroup();

    void addCollisionObject();

private:
    // MoveIt 接口
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

    void drawTitle(const std::string& text);
    void prompt(const std::string& text);
    void drawTrajectory(const moveit::planning_interface::MoveGroupInterface::Plan& plan);

};

#endif
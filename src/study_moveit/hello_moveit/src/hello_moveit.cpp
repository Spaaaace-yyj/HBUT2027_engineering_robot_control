#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "../include/hello_moveit/hello_moveit.hpp"

HelloMoveIt::HelloMoveIt()
    : Node("hello_moveit_node")
{
    RCLCPP_INFO(this->get_logger(), "HelloMoveIt node started");

}

void HelloMoveIt::initMoveGroup()
{
    //关键：MoveGroupInterface 需要 shared_from_this()
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(),
        "panda_arm"
    );


    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        shared_from_this(),
        "panda_link0",
        rviz_visual_tools::RVIZ_MARKER_TOPIC,
        move_group_->getRobotModel()
    );

    visual_tools_->deleteAllMarkers();
    visual_tools_->loadRemoteControl();
}

void HelloMoveIt::drawTitle(const std::string& text)
{
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation().z() = 1.0;

    visual_tools_->publishText(
        pose, text,
        rviz_visual_tools::WHITE,
        rviz_visual_tools::XLARGE
    );
}

void HelloMoveIt::prompt(const std::string& text)
{
    visual_tools_->prompt(text);
}

void HelloMoveIt::drawTrajectory(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    auto jmg = move_group_->getRobotModel()->getJointModelGroup("rm_robot_arm");
    visual_tools_->publishTrajectoryLine(plan.trajectory_, jmg);
}


void HelloMoveIt::runDemo()
{
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.y = 0.8;
    target_pose.orientation.w = 0.6;
    target_pose.position.x = 0.1;
    target_pose.position.y = 0.4;
    target_pose.position.z = 0.4;

    move_group_->setPoseTarget(target_pose);

    addCollisionObject();

    prompt("Press Next to plan");
    drawTitle("Planning");
    visual_tools_->trigger();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_->plan(plan));

    if (success)
    {
        drawTrajectory(plan);
        visual_tools_->trigger();

        prompt("Press Next to execute");
        drawTitle("Executing");
        visual_tools_->trigger();

        move_group_->execute(plan);
    }
    else
    {
        drawTitle("Planning Failed");
        visual_tools_->trigger();
        RCLCPP_ERROR(this->get_logger(), "Planning failed");
    }
}

void HelloMoveIt::addCollisionObject()
{
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    moveit_msgs::msg::CollisionObject collision_object;

    collision_object.header.frame_id = move_group_->getPlanningFrame();
    collision_object.id = "box1";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.2;
    box_pose.position.y = 0.2;
    box_pose.position.z = 0.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    planning_scene_interface.applyCollisionObject(collision_object);
    RCLCPP_INFO(this->get_logger(), "Collision object added");
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<HelloMoveIt>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner([&executor]() { executor.spin(); });

    node->initMoveGroup();
    node->runDemo();

    rclcpp::shutdown();
    spinner.join();

    return 0;
}
//
// Created by spaaaaace on 2026/4/26.
//

#include "../include/engineering_hardware/engineering_system.h"

using namespace engineering_hardware;

hardware_interface::CallbackReturn EngineeringSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_positions_.resize(info.joints.size(), 0.0);
    hw_commands_.resize(info.joints.size(), 0.0);
    hw_velocities_.resize(info.joints.size(), 0.0);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EngineeringSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
    for (size_t i = 0; i < hw_positions_.size(); i++)
    {
        hw_positions_[i] = 0.0;
        hw_commands_[i] = 0.0;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
EngineeringSystem::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> states;

    for (size_t i = 0; i < hw_positions_.size(); i++)
    {
        states.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
        states.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    }

    return states;
}

std::vector<hardware_interface::CommandInterface>
EngineeringSystem::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> commands;

    for (size_t i = 0; i < hw_commands_.size(); i++)
    {
        commands.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
    }

    return commands;
}

hardware_interface::CallbackReturn EngineeringSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
    for (size_t i = 0; i < hw_positions_.size(); i++)
    {
        hw_commands_[i] = hw_positions_[i];
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

//从单片机接受，然后发送给moveit2
hardware_interface::return_type EngineeringSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
    for (size_t i = 0; i < hw_positions_.size(); i++)
    {
        hw_velocities_[i] = hw_commands_[i] - hw_positions_[i];
        hw_positions_[i] = hw_commands_[i];
    }

    return hardware_interface::return_type::OK;
}

//发送给单片机的控制指令
hardware_interface::return_type EngineeringSystem::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
    for (size_t i = 0; i < hw_commands_.size(); i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("hardware"),
                    "joint %ld cmd: %f", i, hw_commands_[i]);
    }

    return hardware_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  engineering_hardware::EngineeringSystem,
  hardware_interface::SystemInterface
)
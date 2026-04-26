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

    init_param();

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

    init_serial();

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

hardware_interface::CallbackReturn EngineeringSystem::on_deactivate(
    const rclcpp_lifecycle::State& previous_state)
{
    running_ = false;
    if (receive_thread_.joinable())
    {
        receive_thread_.join();
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
    size_t size = 5 * sizeof(float);
    std::vector<uint8_t> buffer(size);
    std::vector<float> send_float(hw_commands_.size());
    for (size_t i = 0; i < hw_commands_.size(); i++)
    {
        RCLCPP_INFO(rclcpp::get_logger("hardware"),
                    "joint %ld cmd: %f", i, hw_commands_[i]);
        send_float[i] = hw_commands_[i];

    }
    buffer = floatArrayToBuffer(send_float);

    std::stringstream ss;
    for (size_t i = 0; i < 5 * sizeof(float); ++i)
    {
        ss << std::hex << std::setw(2) << std::setfill('0')
           << static_cast<int>(buffer[i]) << " ";
    }
    RCLCPP_INFO(rclcpp::get_logger("buffer"), "%s", ss.str().c_str());

    if (serial_driver_->port()->is_open())
    {
        serial_driver_->port()->send(buffer);

    }else
    {
        RCLCPP_ERROR(rclcpp::get_logger("buffer"), "open serial fail!");
    }

    return hardware_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  engineering_hardware::EngineeringSystem,
  hardware_interface::SystemInterface
)

void EngineeringSystem::init_param()
{
    using FlowControl = drivers::serial_driver::FlowControl;
    using Parity = drivers::serial_driver::Parity;
    using StopBits = drivers::serial_driver::StopBits;

    uint32_t baud_rate{};
    auto fc = FlowControl::NONE;
    auto pt = Parity::NONE;
    auto sb = StopBits::ONE;

    try
    {
        device_name_ = info_.hardware_parameters.at("device_name");
    }
    catch (const std::exception & ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("hardware"), "The device name provided was invalid");
        throw ex;
    }

    try
    {
        baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    }
    catch (const std::exception & ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("hardware"), "The baud_rate provided was invalid");
        throw ex;
    }

    try
    {
        const auto fc_string = info_.hardware_parameters["fc_string"];

        if (fc_string == "none")
        {
            fc = FlowControl::NONE;
        }
        else if (fc_string == "hardware")
        {
            fc = FlowControl::HARDWARE;
        }
        else if (fc_string == "software")
        {
            fc = FlowControl::SOFTWARE;
        }
        else
        {
            throw std::invalid_argument{ "The flow_control parameter must be one of: none, software, or hardware." };
        }
    }
    catch (const std::exception & ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("hardware"), "The flow_control provided was invalid");
        throw ex;
    }

    try
    {
        const auto pt_string = info_.hardware_parameters["pt_string"];

        if (pt_string == "none")
        {
            pt = Parity::NONE;
        }
        else if (pt_string == "odd")
        {
            pt = Parity::ODD;
        }
        else if (pt_string == "even")
        {
            pt = Parity::EVEN;
        }
        else
        {
            throw std::invalid_argument{ "The parity parameter must be one of: none, odd, or even." };
        }
    }
    catch (const std::exception & ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("hardware"), "The parity provided was invalid");
        throw ex;
    }

    try
    {
        const auto sb_string = info_.hardware_parameters["sb_string"];

        if (sb_string == "1" || sb_string == "1.0")
        {
            sb = StopBits::ONE;
        }
        else if (sb_string == "1.5")
        {
            sb = StopBits::ONE_POINT_FIVE;
        }
        else if (sb_string == "2" || sb_string == "2.0")
        {
            sb = StopBits::TWO;
        }
        else
        {
            throw std::invalid_argument{ "The stop_bits parameter must be one of: 1, 1.5, or 2." };
        }
    }
    catch (const std::exception & ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("hardware"), "The stop_bits provided was invalid");
        throw ex;
    }

    device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void EngineeringSystem::init_serial()
{

    owned_ctx_ = std::make_unique<IoContext>(2);
    serial_driver_ = std::make_unique<drivers::serial_driver::SerialDriver>(*owned_ctx_);
    try
    {
        serial_driver_->init_port(device_name_, *device_config_);
        if (!serial_driver_->port()->is_open())
        {
            serial_driver_->port()->open();
            running_ = true;
            receive_thread_ = std::thread(&EngineeringSystem::serial_recv, this);
        }
    }
    catch (const std::exception& ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("lc_serial"), "Error creating lc_serial port: %s - %s", device_name_.c_str(), ex.what());
        // throw ex;
    }
}

void EngineeringSystem::bufferToFloatArray(const uint8_t* buffer, float* floatArray, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        memcpy(&floatArray[i], buffer + i * sizeof(float), sizeof(float));
    }
}

std::vector<uint8_t> EngineeringSystem::floatArrayToBuffer(
    const std::vector<float>& data)
{
    std::vector<uint8_t> buffer(data.size() * sizeof(float));

    memcpy(buffer.data(), data.data(), buffer.size());

    return buffer;
}

void EngineeringSystem::serial_recv()
{
    while (rclcpp::ok())
    {
        // try
        // {
        //
        // }catch (const std::exception& ex)
        // {
        //
        // }

    }
}

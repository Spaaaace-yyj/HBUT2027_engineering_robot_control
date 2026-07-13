//
// Created by spaaaaace on 2026/4/26.
//

#include "../include/engineering_hardware/engineering_system.h"

#include "engineering_hardware/serial_process.hpp"

#include <iostream>

using namespace engineering_hardware;

//初始化
hardware_interface::CallbackReturn EngineeringSystem::on_init(
    const hardware_interface::HardwareInfo& info)
{
    //ros2node初始化
    node_ = std::make_shared<rclcpp::Node>("engineering_hw_node");
    //debug话题
    debug_pub_ = node_->create_publisher<auto_aim_interfaces::msg::RobotArmDebug>("/hw_debug", 10);

    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_positions_.resize(info.joints.size(), 0.0);
    hw_velocities_.resize(info.joints.size(), 0.0);

    hw_commands_positions_.resize(info.joints.size(), 0.0);
    hw_commands_velocities_.resize(info.joints.size(), 0.0);

    //初始化参数，从硬件描述文件中读取参数
    init_param();

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EngineeringSystem::on_configure(
    const rclcpp_lifecycle::State&)
{
    for (size_t i = 0; i < hw_positions_.size(); i++)
    {
        hw_positions_[i] = 0.0;
        hw_commands_positions_[i] = 0.0;
        hw_commands_velocities_[i] = 0.0;
    }

    //打开串口
    init_serial();

    //ros2线程
    ros_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    ros_executor_->add_node(node_);
    spin_thread_ = std::thread([this]()
    {
        ros_executor_->spin();
    });

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

    for (size_t i = 0; i < hw_commands_positions_.size(); i++)
    {
        commands.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]);
        commands.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]);
    }

    return commands;
}

hardware_interface::CallbackReturn EngineeringSystem::on_activate(
    const rclcpp_lifecycle::State&)
{
    for (size_t i = 0; i < hw_positions_.size(); i++)
    {
        hw_commands_positions_[i] = hw_positions_[i];
        hw_commands_velocities_[i] = hw_velocities_[i];
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

    if (ros_executor_)
    {
        ros_executor_->cancel();
    }

    if (spin_thread_.joinable())
    {
        spin_thread_.join();
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

void EngineeringSystem::serial_recv()
{
    while (running_)
    {
        recv_buffer_.clear();
        recv_buffer_.resize(256);

        size_t n = serial_driver_->port()->receive(recv_buffer_);
        if (n <= 10)
        {
            RCLCPP_ERROR(node_->get_logger(), "Serial receive too short!");
            continue;
        }

        if (recv_buffer_[0] == 0xA5)
        {
            uint16_t flags_register;
            uint16_t decode_state = get_protocol_info(recv_buffer_.data(), &flags_register, (uint8_t*)&recv_data_.joint0);
            if (decode_state != 0x01)
            {
                RCLCPP_ERROR(node_->get_logger(), "The data packet is damaged and cannot be parsed! Cmd id = %x",
                             decode_state);
                continue;
            }
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Receiving serial port frame header error; Header = %x", recv_buffer_[0]);
            continue;
        }
    }
}

//从单片机接受，然后发送给moveit2
hardware_interface::return_type EngineeringSystem::read(
    const rclcpp::Time&,
    const rclcpp::Duration&)
{


    for (size_t i = 0; i < hw_positions_.size(); i++)
    {
        //todo:这里速度后续要不要接上，要不要呢？
        hw_velocities_[i] = hw_commands_velocities_[i];
    }
    hw_positions_[0] = recv_data_.joint1 * -1.0;
    hw_positions_[1] = recv_data_.joint6;
    hw_positions_[2] = recv_data_.joint5;
    hw_positions_[3] = recv_data_.joint2;
    hw_positions_[4] = recv_data_.joint3 * -1.0;
    hw_positions_[5] = recv_data_.joint4;

    return hardware_interface::return_type::OK;
}

//发送给单片机的控制指令
hardware_interface::return_type EngineeringSystem::write(
    const rclcpp::Time&,
    const rclcpp::Duration&)
{
    auto_aim_interfaces::msg::RobotArmDebug debug_msg;

    for (size_t i = 0; i < hw_commands_positions_.size(); i++)
    {
        // RCLCPP_INFO(rclcpp::get_logger("hardware"),
        //             "joint %ld cmd: %f", i, hw_commands_positions_[i]);
        debug_msg.joint.push_back(hw_commands_positions_[i]);
        debug_msg.joint_v.push_back(hw_commands_velocities_[i]);
        send_data_.joint[i] = hw_commands_positions_[i];
    }

    debug_msg.joint[4] *= -1.0f;
    debug_msg.joint[0] *= -1.0f;
    send_data_.joint[4] *= -1.0f;
    send_data_.joint[0] *= -1.0f;
    debug_pub_->publish(debug_msg);

    //序列化
    uint16_t flag_register = 0x0000;
    uint16_t tx_len;
    uint8_t send_temp[64] = {0};

    get_protocol_send_data(0x01, flag_register, send_data_.joint, 6, send_temp, &tx_len);
    std::vector<uint8_t> send_buffer(send_temp, send_temp + tx_len);

    for (size_t i = 0; i < send_buffer.size(); ++i)
    {
        printf("%02X ", send_buffer[i]);
    }
    printf("\n");

    if (serial_driver_ && serial_driver_->port()->is_open())
    {
        serial_driver_->port()->send(send_buffer);
    }
    else
    {
        serial_timeout_counter_++;
        RCLCPP_ERROR(node_->get_logger(), "Disconnect with Serial port! Try to reconnect....");
        OpenPort();
    }


    return hardware_interface::return_type::OK;
}

void EngineeringSystem::OpenPort()
{
    try
    {
        if (serial_driver_->port()->is_open())
        {
            RCLCPP_WARN(node_->get_logger(), "Serial port is open, closing!");
            serial_driver_->port()->close();
        }

        rclcpp::sleep_for(std::chrono::milliseconds(100));

        serial_driver_->port()->open();
        if (serial_driver_->port()->is_open())
        {
            RCLCPP_INFO(node_->get_logger(), "Serial port Open!");
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Serial open faild");
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Serial port reopen failed: %s", e.what());
    }
}

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
    catch (const std::exception& ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("hardware"), "The device name provided was invalid");
        throw ex;
    }

    try
    {
        baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    }
    catch (const std::exception& ex)
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
            throw std::invalid_argument{"The flow_control parameter must be one of: none, software, or hardware."};
        }
    }
    catch (const std::exception& ex)
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
            throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
        }
    }
    catch (const std::exception& ex)
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
            throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
        }
    }
    catch (const std::exception& ex)
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
        RCLCPP_INFO(rclcpp::get_logger("lc_serial"), "success init serial %s", device_name_.c_str());
    }
    catch (const std::exception& ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("lc_serial"), "Error creating lc_serial port: %s - %s", device_name_.c_str(),
                     ex.what());
        // throw ex;
    }
}

void EngineeringSystem::bufferToFloatArray(const uint8_t* buffer, float* floatArray, size_t size)
{
    for (size_t i = 0; i < size; ++i)
    {
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

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    engineering_hardware::EngineeringSystem,
    hardware_interface::SystemInterface
)

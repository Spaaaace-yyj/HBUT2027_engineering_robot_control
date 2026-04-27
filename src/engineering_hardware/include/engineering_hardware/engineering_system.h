//
// Created by spaaaaace on 2026/4/26.
//

#ifndef BUILD_ENGINEERING_SYSTEM_H
#define BUILD_ENGINEERING_SYSTEM_H

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "rclcpp/rclcpp.hpp"

#include <serial_driver/serial_driver.hpp>
#include <iostream>
#include "auto_aim_interfaces/msg/robot_arm_debug.hpp"

namespace engineering_hardware
{
    class EngineeringSystem : public hardware_interface::SystemInterface
    {
    public:
        //初始化
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        //lifecycle生命周期管理
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::return_type read(
          const rclcpp::Time & time,
          const rclcpp::Duration & period) override;

        hardware_interface::return_type write(
          const rclcpp::Time & time,
          const rclcpp::Duration & period) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& previous_state) override;

        void init_param();

        void init_serial();

        void serial_recv();

        void bufferToFloatArray(const uint8_t* buffer, float* floatArray, size_t size);

        std::vector<uint8_t> floatArrayToBuffer(const std::vector<float>& data);
    private:
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;
        std::vector<double> hw_commands_positions_;
        std::vector<double> hw_commands_velocities_;

        std::unique_ptr<IoContext> owned_ctx_;
        std::string device_name_;
        std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
        std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

        std::thread receive_thread_;
        std::atomic<bool> running_{false};
        std::mutex data_mutex_;

        //debug
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<auto_aim_interfaces::msg::RobotArmDebug>::SharedPtr debug_pub_;
        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> ros_executor_;
        std::thread spin_thread_;

    };

}

#endif //BUILD_ENGINEERING_SYSTEM_H
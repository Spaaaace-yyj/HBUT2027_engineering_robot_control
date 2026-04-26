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

    private:
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;
        std::vector<double> hw_commands_;
    };

}

#endif //BUILD_ENGINEERING_SYSTEM_H
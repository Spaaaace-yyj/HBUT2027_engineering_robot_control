cmake_minimum_required(VERSION 3.8)
project(engineering_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(serial_driver REQUIRED)
find_package(auto_aim_interfaces REQUIRED)

add_library(engineering_hardware SHARED
        src/engineering_system.cpp
        src/crc_check.cpp
        src/serial_process.cpp
)

target_include_directories(engineering_hardware PUBLIC
        include
)

ament_target_dependencies(engineering_hardware
        rclcpp
        hardware_interface
        pluginlib
        serial_driver
        auto_aim_interfaces
)

pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

install(TARGETS engineering_hardware
        DESTINATION lib
)

install(DIRECTORY include/
        DESTINATION include
)

install(FILES plugin.xml
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

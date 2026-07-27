cmake_minimum_required(VERSION 3.8)
project(rm_vision_pick_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)

add_executable(vision_pick_place_node src/vision_pick_place_node.cpp)
target_compile_features(vision_pick_place_node PUBLIC cxx_std_17)
ament_target_dependencies(vision_pick_place_node
  rclcpp std_msgs std_srvs geometry_msgs shape_msgs
  tf2 tf2_ros tf2_geometry_msgs
  moveit_msgs moveit_ros_planning_interface moveit_task_constructor_core)

install(TARGETS vision_pick_place_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

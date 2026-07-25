cmake_minimum_required(VERSION 3.8)
project(rm_handeye_calibration)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(handeye_calibration_node
  src/handeye_calibration_node.cpp
)
ament_target_dependencies(handeye_calibration_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  tf2
  tf2_ros
)
target_include_directories(handeye_calibration_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(handeye_calibration_node
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

add_executable(generate_charuco_board
  src/generate_charuco_board.cpp
)
target_include_directories(generate_charuco_board PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(generate_charuco_board
  ${OpenCV_LIBS}
)

install(TARGETS
  handeye_calibration_node
  generate_charuco_board
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md LICENSE
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

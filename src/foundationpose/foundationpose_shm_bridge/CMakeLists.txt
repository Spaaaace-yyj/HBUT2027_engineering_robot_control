cmake_minimum_required(VERSION 3.8)
project(foundationpose_shm_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(foundationpose_shm_ipc
  src/shared_rgbd_writer.cpp
  src/control_shm_writer.cpp
  src/result_shm_reader.cpp
)

target_include_directories(foundationpose_shm_ipc
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(foundationpose_shm_ipc
  ${OpenCV_LIBS}
  rt
)

add_executable(zed_shm_bridge_node
  src/zed_shm_bridge_node.cpp
)

target_include_directories(zed_shm_bridge_node
  PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(zed_shm_bridge_node
  rclcpp
  rcl_interfaces
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  cv_bridge
  message_filters
  tf2
  tf2_ros
  visualization_msgs
)

target_link_libraries(zed_shm_bridge_node
  foundationpose_shm_ipc
  ${OpenCV_LIBS}
  rt
)

install(
  TARGETS
    foundationpose_shm_ipc
    zed_shm_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

install(
        DIRECTORY mesh
        DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(foundationpose_shm_ipc)
ament_package()

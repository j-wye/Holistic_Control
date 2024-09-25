cmake_minimum_required(VERSION 3.5)
project(mobile_manipulator)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
find_package(spdlog REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(console_bridge REQUIRED)

# include_directories(ros2_kortex_driver/include)

install(
  DIRECTORY
  config
  launch
  meshes
  rviz
  scripts
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

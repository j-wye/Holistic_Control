cmake_minimum_required(VERSION 3.5)
project(realsense_gazebo_plugin)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo REQUIRED)

set(CMAKE_CXX_STANDARD 14) # Ensure C++14 standard for compatibility with Gazebo and ROS2

# Include directories
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

# Link directories
link_directories(${GAZEBO_LIBRARY_DIRS})

# Create the library from the source files
add_library(${PROJECT_NAME} SHARED
  src/RealSensePlugin.cpp
  src/gazebo_ros_realsense.cpp
)

# Link libraries
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  gazebo_ros
  image_transport
  camera_info_manager
  sensor_msgs
  gazebo
)

target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES})

# Install the library
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/
)

# Ament package
ament_package()


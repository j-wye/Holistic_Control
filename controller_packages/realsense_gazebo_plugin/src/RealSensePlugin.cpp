#include "realsense_gazebo_plugin/RealSensePlugin.h"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gazebo {

RealSensePlugin::RealSensePlugin() {}

RealSensePlugin::~RealSensePlugin() {}

void RealSensePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  auto node = rclcpp::Node::make_shared("realsense_node");

  this->depthPub = node->create_publisher<sensor_msgs::msg::Image>("camera/depth/image_raw", 10);
  this->colorPub = node->create_publisher<sensor_msgs::msg::Image>("camera/color/image_raw", 10);
  this->ired1Pub = node->create_publisher<sensor_msgs::msg::Image>("camera/ir1/image_raw", 10);
  this->ired2Pub = node->create_publisher<sensor_msgs::msg::Image>("camera/ir2/image_raw", 10);
}

void RealSensePlugin::OnNewDepthFrame() {
  auto node = rclcpp::Node::make_shared("realsense_node");
  sensor_msgs::msg::Image depth_image_msg;
  depth_image_msg.header.stamp = node->now();
  depth_image_msg.height = this->depthCam->ImageHeight();
  depth_image_msg.width = this->depthCam->ImageWidth();
  depth_image_msg.encoding = sensor_msgs::image_encodings::MONO16;
  depth_image_msg.step = sizeof(uint16_t) * this->depthCam->ImageWidth();
  
  unsigned int imageSize = this->depthCam->ImageHeight() * this->depthCam->ImageWidth();
  depth_image_msg.data.resize(imageSize * sizeof(uint16_t));

  const float *depthDataFloat = this->depthCam->DepthData();
  for (unsigned int i = 0; i < imageSize; ++i) {
    if (depthDataFloat[i] < rangeMinDepth_ || depthDataFloat[i] > rangeMaxDepth_) {
      reinterpret_cast<uint16_t &>(depth_image_msg.data[i * sizeof(uint16_t)]) = 0;
    } else {
      reinterpret_cast<uint16_t &>(depth_image_msg.data[i * sizeof(uint16_t)]) = static_cast<uint16_t>(depthDataFloat[i] * 1000);
    }
  }

  this->depthPub->publish(depth_image_msg);
}

void RealSensePlugin::OnUpdate() {}
}
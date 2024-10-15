#include "realsense_gazebo_plugin/gazebo_ros_realsense.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

namespace gazebo {

GazeboRosRealsense::GazeboRosRealsense() : RealSensePlugin() {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  this->rosnode_ = rclcpp::Node::make_shared("gazebo_ros_realsense");
  this->itnode_ = std::make_shared<image_transport::ImageTransport>(this->rosnode_);
}

GazeboRosRealsense::~GazeboRosRealsense() {
  rclcpp::shutdown();
}

void GazeboRosRealsense::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  RealSensePlugin::Load(_model, _sdf);

  this->color_pub_ = this->itnode_->advertiseCamera("camera/color/image_raw", 1);
  this->depth_pub_ = this->itnode_->advertiseCamera("camera/depth/image_raw", 1);
  this->pointcloud_pub_ = this->rosnode_->create_publisher<sensor_msgs::msg::PointCloud2>("camera/depth/points", 1);
}

void GazeboRosRealsense::OnNewDepthFrame() {
  rclcpp::Time current_time = this->rosnode_->now();
  this->depth_msg_.header.frame_id = this->cameraParamsMap_[DEPTH_CAMERA_NAME].optical_frame;
  this->depth_msg_.header.stamp = current_time;

  std::string pixel_format = sensor_msgs::image_encodings::TYPE_16UC1;
  this->depth_msg_.height = this->depthCam->ImageHeight();
  this->depth_msg_.width = this->depthCam->ImageWidth();
  this->depth_msg_.encoding = pixel_format;
  this->depth_msg_.step = 2 * this->depthCam->ImageWidth();
  this->depth_msg_.data.resize(2 * this->depthCam->ImageWidth() * this->depthCam->ImageHeight());

  memcpy(this->depth_msg_.data.data(), this->depthMap.data(), 2 * this->depthCam->ImageWidth() * this->depthCam->ImageHeight());

  auto depth_info_msg = this->cameraInfo(this->depth_msg_, this->depthCam->HFOV().Radian());
  this->depth_pub_.publish(this->depth_msg_, depth_info_msg);

  if (pointCloud_ && this->pointcloud_pub_->get_subscription_count() > 0) {
    this->pointcloud_msg_.header = this->depth_msg_.header;
    this->pointcloud_msg_.width = this->depthCam->ImageWidth();
    this->pointcloud_msg_.height = this->depthCam->ImageHeight();
    this->pointcloud_msg_.row_step = this->pointcloud_msg_.point_step * this->depthCam->ImageWidth();
    this->FillPointCloudHelper(this->pointcloud_msg_, this->depthCam->ImageHeight(), this->depthCam->ImageWidth(), 2 * this->depthCam->ImageWidth(), (void *)this->depthCam->DepthData());
    this->pointcloud_pub_->publish(this->pointcloud_msg_);
  }
}

bool GazeboRosRealsense::FillPointCloudHelper(sensor_msgs::msg::PointCloud2 &point_cloud_msg, uint32_t rows_arg, uint32_t cols_arg, uint32_t step_arg, void *data_arg) {
  sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(point_cloud_msg, "rgb");

  for (uint32_t i = 0; i < rows_arg; ++i) {
    for (uint32_t j = 0; j < cols_arg; ++j, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
      float depth_value = static_cast<float *>(data_arg)[i * cols_arg + j];
      *iter_x = j * depth_value;
      *iter_y = i * depth_value;
      *iter_z = depth_value;
      iter_rgb[0] = 255;
      iter_rgb[1] = 255;
      iter_rgb[2] = 255;
    }
  }

  point_cloud_msg.height = rows_arg;
  point_cloud_msg.width = cols_arg;
  point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width;

  return true;
}

sensor_msgs::msg::CameraInfo GazeboRosRealsense::cameraInfo(const sensor_msgs::msg::Image &image, double fov) {
  sensor_msgs::msg::CameraInfo info_msg;
  info_msg.header = image.header;
  info_msg.height = image.height;
  info_msg.width = image.width;

  double fx = image.width / (2.0 * tan(fov / 2.0));
  double fy = fx;

  info_msg.k[0] = fx;
  info_msg.k[2] = image.width / 2.0;
  info_msg.k[4] = fy;
  info_msg.k[5] = image.height / 2.0;
  info_msg.k[8] = 1.0;

  info_msg.p[0] = info_msg.k[0];
  info_msg.p[2] = info_msg.k[2];
  info_msg.p[5] = info_msg.k[4];
  info_msg.p[6] = info_msg.k[5];
  info_msg.p[10] = info_msg.k[8];

  return info_msg;
}

}
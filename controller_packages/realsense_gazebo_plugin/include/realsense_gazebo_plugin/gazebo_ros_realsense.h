#ifndef _GAZEBO_ROS_REALSENSE_PLUGIN_
#define _GAZEBO_ROS_REALSENSE_PLUGIN_

#include "realsense_gazebo_plugin/RealSensePlugin.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

#include <memory>
#include <string>

namespace gazebo {
class GazeboRosRealsense : public RealSensePlugin {
public:
  GazeboRosRealsense();
  ~GazeboRosRealsense();

  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  virtual void OnNewDepthFrame() override;

  bool FillPointCloudHelper(sensor_msgs::msg::PointCloud2 &point_cloud_msg, uint32_t rows_arg,
                            uint32_t cols_arg, uint32_t step_arg, void *data_arg);

  virtual void OnNewFrame(const rendering::CameraPtr cam,
                          const transport::PublisherPtr pub) override;

  sensor_msgs::msg::CameraInfo cameraInfo(const sensor_msgs::msg::Image &image, double fov);

protected:
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  rclcpp::Node::SharedPtr rosnode_;

private:
  std::shared_ptr<image_transport::ImageTransport> itnode_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

protected:
  image_transport::CameraPublisher color_pub_, ir1_pub_, ir2_pub_, depth_pub_;
  sensor_msgs::msg::Image image_msg_, depth_msg_;
  sensor_msgs::msg::PointCloud2 pointcloud_msg_;
};
}
#endif
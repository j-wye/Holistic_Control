#ifndef _GZRS_PLUGIN_HH_
#define _GZRS_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>
#include <sdf/sdf.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>

namespace gazebo {
#define DEPTH_CAMERA_NAME "depth"
#define COLOR_CAMERA_NAME "color"
#define IRED1_CAMERA_NAME "ired1"
#define IRED2_CAMERA_NAME "ired2"

struct CameraParams {
  CameraParams() {}

  std::string topic_name;
  std::string camera_info_topic_name;
  std::string optical_frame;
};

class RealSensePlugin : public ModelPlugin {
public:
  RealSensePlugin();
  ~RealSensePlugin();

  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void OnUpdate();
  virtual void OnNewDepthFrame();
  virtual void OnNewFrame(const rendering::CameraPtr cam,
                          const transport::PublisherPtr pub);

protected:
  physics::ModelPtr rsModel;
  physics::WorldPtr world;
  rendering::DepthCameraPtr depthCam;
  rendering::CameraPtr colorCam;
  rendering::CameraPtr ired1Cam;
  rendering::CameraPtr ired2Cam;
  std::string prefix;
  transport::NodePtr transportNode;
  std::vector<uint16_t> depthMap;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthPub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr colorPub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ired1Pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ired2Pub;
  event::ConnectionPtr newDepthFrameConn;
  event::ConnectionPtr newIred1FrameConn;
  event::ConnectionPtr newIred2FrameConn;
  event::ConnectionPtr newColorFrameConn;
  event::ConnectionPtr updateConnection;
  std::map<std::string, CameraParams> cameraParamsMap_;
  bool pointCloud_ = false;
  std::string pointCloudTopic_;
  double pointCloudCutOff_, pointCloudCutOffMax_;
  double colorUpdateRate_;
  double infraredUpdateRate_;
  double depthUpdateRate_;
  float rangeMinDepth_;
  float rangeMaxDepth_;
};
}
#endif
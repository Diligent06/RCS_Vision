#ifndef MINDVISION_CAMERA_HPP
#define MINDVISION_CAMERA_HPP
// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

// MindVision Camera SDK
#include <CameraApi.h>

#include "opencv2/opencv.hpp"

// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include "std_msgs/msg/string.hpp"

// C++ system
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <chrono>
#define image_width 960
#define image_height 768
namespace mindvision_camera
{
class MVCameraNode : public rclcpp::Node
{
public:
  explicit MVCameraNode(const rclcpp::NodeOptions & options=rclcpp::NodeOptions().use_intra_process_comms(true));

  ~MVCameraNode() override
  {
    CameraUnInit(h_camera_);
    videowriter.release();
    RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
  }
  void timer_callback();
private:
  void declareParameters();

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  int h_camera_;
  uint8_t * pby_buffer_;
  tSdkCameraCapbility t_capability_;  // 设备描述信息
  tSdkFrameHead s_frame_info_;        // 图像帧头信息

  sensor_msgs::msg::Image image_temp;
  // for video recorder
  cv_bridge::CvImagePtr cv_ptr;

  image_transport::CameraPublisher camera_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

  // RGB Gain
  int r_gain_, g_gain_, b_gain_;
  
  // flag of image flip
  bool flip_image_;
  // 定义发布的摄像头信息
  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  // 参数改变的回调函数
  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
  // 设置图像传输速度与曝光时间
  typedef enum transmission_speed_ {Normal, Fast} transmission_speed_;
  transmission_speed_ transmission_speed = Fast ;
  bool auto_expose = true;
  const int exposure_time = 5000;
  int channel = 3;
  // 视频记录器，record=1则开启记录视频图像，否则不记录
  cv::VideoWriter videowriter; 
  int record = true;
  // callback group and timer declare
  rclcpp::CallbackGroup::SharedPtr timer_cb_group;
  rclcpp::TimerBase::SharedPtr timer;
};

}  // namespace mindvision_camera
#endif
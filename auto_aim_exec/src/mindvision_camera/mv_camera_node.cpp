#include "mindvision_camera/mindvision_camera.hpp"
namespace mindvision_camera
{
MVCameraNode::MVCameraNode(const rclcpp::NodeOptions & options) : Node("mv_camera", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting MVCameraNode!");

  CameraSdkInit(1);

  // 枚举设备，并建立设备列表
  int i_camera_counts = 1;
  int i_status = -1;
  tSdkCameraDevInfo t_camera_enum_list;
  i_status = CameraEnumerateDevice(&t_camera_enum_list, &i_camera_counts);
  RCLCPP_INFO(this->get_logger(), "Enumerate state = %d", i_status);
  RCLCPP_INFO(this->get_logger(), "Found camera count = %d", i_camera_counts);

  // 没有连接设备
  if (i_camera_counts == 0) {
    RCLCPP_ERROR(this->get_logger(), "No camera found!");
    return;
  }

  // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
  i_status = CameraInit(&t_camera_enum_list, -1, -1, &h_camera_);

  // 初始化失败
  RCLCPP_INFO(this->get_logger(), "Init state = %d", i_status);
  if (i_status != CAMERA_STATUS_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Init failed!");
    return;
  }

  // 获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
  CameraGetCapability(h_camera_, &t_capability_);

  // 让SDK进入工作模式，开始接收来自相机发送的图像
  // 数据。如果当前相机是触发模式，则需要接收到
  // 触发帧以后才会更新图像。
  CameraPlay(h_camera_);

  // 设置相机采集通道数
  if (t_capability_.sIspCapacity.bMonoSensor) {
    channel = 1;
    CameraSetIspOutFormat(h_camera_, CAMERA_MEDIA_TYPE_MONO8);
  } else {
    channel = 3;
    CameraSetIspOutFormat(h_camera_, CAMERA_MEDIA_TYPE_BGR8);
  }

  // Set Camera Frame Speed
  if (CameraSetFrameSpeed(h_camera_, t_capability_.iFrameSpeedDesc - 1) != CAMERA_STATUS_SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "Camera speed set to the fastest!");
  } else {
    RCLCPP_INFO(this->get_logger(), "Camera speed set wrong!");
  }

  // 设置自动曝光
  auto_expose = false;
  if (auto_expose)
    CameraSetAeState(h_camera_, true);
  else
    CameraSetAeState(h_camera_, false);

  // Declare camera parameters
  declareParameters();

  // Set camera resolution
  tSdkImageResolution roi = {0};
  roi.iIndex = 0xff;
  roi.iWidth = 960;
  roi.iWidthFOV = 1280;
  roi.iHeight = 768;
  roi.iHeightFOV = 1024;
  roi.iHOffsetFOV = 160;
  roi.iVOffsetFOV = 128;
  if (CameraSetImageResolution(h_camera_, &roi) == CAMERA_STATUS_SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "Successful set camera resolution");
  }
  videowriter.open("record.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, cv::Size(roi.iWidth, roi.iHeight), true);
  if(!videowriter.isOpened())
  {
    RCLCPP_WARN(this->get_logger(), "Open videowriter failed!");
  }
  // Create camera publisher
  pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 1);
  // Load camera info
  camera_name_ = this->declare_parameter("camera_name", "mv_camera");
  camera_info_manager_ =
    std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
  auto camera_info_url = this->declare_parameter(
    "camera_info_url", "package://auto_aim_exec/config/mindvision_camera/camera_info.yaml");
  if (camera_info_manager_->validateURL(camera_info_url)) {
    camera_info_manager_->loadCameraInfo(camera_info_url);
    camera_info_msg_ = camera_info_manager_->getCameraInfo();
  } else {
    RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
  }
  
  // Add callback to the set parameter event
  params_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&MVCameraNode::parametersCallback, this, std::placeholders::_1));
  // reserve space for image_temp
  image_temp.data.reserve(
    t_capability_.sResolutionRange.iHeightMax * t_capability_.sResolutionRange.iWidthMax * 3);
  // init image_temp variant
  image_temp.header.frame_id = "camera_optical_frame";
  image_temp.encoding = "rgb8";
  image_temp.height = roi.iHeight;
  image_temp.width = roi.iWidth;
  image_temp.step = roi.iWidth * 3;
  // set callback group and callback function
  timer_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  // call back action every 10ms
  timer = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&MVCameraNode::timer_callback, this));
}
void MVCameraNode::timer_callback()
{
  // UniquePtr for zero copy transmit
  sensor_msgs::msg::Image::UniquePtr image_msg_(new sensor_msgs::msg::Image());
  if (
    CameraGetImageBuffer(h_camera_, &s_frame_info_, &pby_buffer_, 1000) == CAMERA_STATUS_SUCCESS) {
    CameraImageProcess(h_camera_, pby_buffer_, image_temp.data.data(), &s_frame_info_);
    if (flip_image_) {
      CameraFlipFrameBuffer(image_temp.data.data(), &s_frame_info_, 3);
    }
    image_temp.data.resize(s_frame_info_.iWidth * s_frame_info_.iHeight * 3);
    // copy image data from image_temp
    image_msg_->data = image_temp.data;
    image_msg_->height = s_frame_info_.iHeight;
    image_msg_->width = s_frame_info_.iWidth;
    image_msg_->step = s_frame_info_.iWidth * 3;
    image_msg_->data.resize(s_frame_info_.iWidth * s_frame_info_.iHeight * 3);
    image_msg_->header.frame_id = "camera_optical_frame";
    image_msg_->encoding = "rgb8";
    cv_ptr = cv_bridge::toCvCopy(image_temp, sensor_msgs::image_encodings::TYPE_8UC3);
    if(record)
      videowriter.write(cv_ptr->image);
    pub_->publish(std::move(image_msg_));
    // release image buffer
    // RCLCPP_INFO(this->get_logger(), "camera publish");
    CameraReleaseImageBuffer(h_camera_, pby_buffer_);
  }
}
void MVCameraNode::declareParameters()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;

  // Exposure time
  param_desc.description = "Exposure time in microseconds";
  // 对于CMOS传感器，其曝光的单位是按照行来计算的
  double exposure_line_time;
  CameraGetExposureLineTime(h_camera_, &exposure_line_time);
  param_desc.integer_range[0].from_value =
    t_capability_.sExposeDesc.uiExposeTimeMin * exposure_line_time;
  param_desc.integer_range[0].to_value =
    t_capability_.sExposeDesc.uiExposeTimeMax * exposure_line_time;

  if (!auto_expose) {
    double exposure_time = this->declare_parameter("exposure_time", 5000, param_desc);
    CameraSetExposureTime(h_camera_, exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure time = %f", exposure_time);
  }
  // Analog gain
  param_desc.description = "Analog gain";
  param_desc.integer_range[0].from_value = t_capability_.sExposeDesc.uiAnalogGainMin;
  param_desc.integer_range[0].to_value = t_capability_.sExposeDesc.uiAnalogGainMax;
  int analog_gain;
  CameraGetAnalogGain(h_camera_, &analog_gain);
  analog_gain = this->declare_parameter("analog_gain", analog_gain, param_desc);
  CameraSetAnalogGain(h_camera_, analog_gain);
  RCLCPP_INFO(this->get_logger(), "Analog gain = %d", analog_gain);

  // RGB Gain
  // Get default value
  CameraGetGain(h_camera_, &r_gain_, &g_gain_, &b_gain_);
  // R Gain
  param_desc.integer_range[0].from_value = t_capability_.sRgbGainRange.iRGainMin;
  param_desc.integer_range[0].to_value = t_capability_.sRgbGainRange.iRGainMax;
  r_gain_ = this->declare_parameter("rgb_gain.r", r_gain_, param_desc);
  // G Gain
  param_desc.integer_range[0].from_value = t_capability_.sRgbGainRange.iGGainMin;
  param_desc.integer_range[0].to_value = t_capability_.sRgbGainRange.iGGainMax;
  g_gain_ = this->declare_parameter("rgb_gain.g", g_gain_, param_desc);
  // B Gain
  param_desc.integer_range[0].from_value = t_capability_.sRgbGainRange.iBGainMin;
  param_desc.integer_range[0].to_value = t_capability_.sRgbGainRange.iBGainMax;
  b_gain_ = this->declare_parameter("rgb_gain.b", b_gain_, param_desc);
  // Set gain
  CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
  RCLCPP_INFO(this->get_logger(), "RGB Gain: R = %d", r_gain_);
  RCLCPP_INFO(this->get_logger(), "RGB Gain: G = %d", g_gain_);
  RCLCPP_INFO(this->get_logger(), "RGB Gain: B = %d", b_gain_);

  // Saturation
  param_desc.description = "Saturation";
  param_desc.integer_range[0].from_value = t_capability_.sSaturationRange.iMin;
  param_desc.integer_range[0].to_value = t_capability_.sSaturationRange.iMax;
  int saturation;
  CameraGetSaturation(h_camera_, &saturation);
  saturation = this->declare_parameter("saturation", saturation, param_desc);
  CameraSetSaturation(h_camera_, saturation);
  RCLCPP_INFO(this->get_logger(), "Saturation = %d", saturation);

  // Gamma
  param_desc.integer_range[0].from_value = t_capability_.sGammaRange.iMin;
  param_desc.integer_range[0].to_value = t_capability_.sGammaRange.iMax;
  int gamma;
  CameraGetGamma(h_camera_, &gamma);
  gamma = this->declare_parameter("gamma", gamma, param_desc);
  CameraSetGamma(h_camera_, gamma);
  RCLCPP_INFO(this->get_logger(), "Gamma = %d", gamma);

  // Flip
  flip_image_ = this->declare_parameter("flip_image", false);
}
rcl_interfaces::msg::SetParametersResult MVCameraNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & param : parameters) {
    if (param.get_name() == "exposure_time") {
      int status = CameraSetExposureTime(h_camera_, param.as_int());
      if (status != CAMERA_STATUS_SUCCESS) {
        result.successful = false;
        result.reason = "Failed to set exposure time, status = " + std::to_string(status);
      }
    } else if (param.get_name() == "analog_gain") {
      int status = CameraSetAnalogGain(h_camera_, param.as_int());
      if (status != CAMERA_STATUS_SUCCESS) {
        result.successful = false;
        result.reason = "Failed to set analog gain, status = " + std::to_string(status);
      }
    } else if (param.get_name() == "rgb_gain.r") {
      r_gain_ = param.as_int();
      int status = CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
      if (status != CAMERA_STATUS_SUCCESS) {
        result.successful = false;
        result.reason = "Failed to set RGB gain, status = " + std::to_string(status);
      }
    } else if (param.get_name() == "rgb_gain.g") {
      g_gain_ = param.as_int();
      int status = CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
      if (status != CAMERA_STATUS_SUCCESS) {
        result.successful = false;
        result.reason = "Failed to set RGB gain, status = " + std::to_string(status);
      }
    } else if (param.get_name() == "rgb_gain.b") {
      b_gain_ = param.as_int();
      int status = CameraSetGain(h_camera_, r_gain_, g_gain_, b_gain_);
      if (status != CAMERA_STATUS_SUCCESS) {
        result.successful = false;
        result.reason = "Failed to set RGB gain, status = " + std::to_string(status);
      }
    } else if (param.get_name() == "saturation") {
      int status = CameraSetSaturation(h_camera_, param.as_int());
      if (status != CAMERA_STATUS_SUCCESS) {
        result.successful = false;
        result.reason = "Failed to set saturation, status = " + std::to_string(status);
      }
    } else if (param.get_name() == "gamma") {
      int gamma = param.as_int();
      int status = CameraSetGamma(h_camera_, gamma);
      if (status != CAMERA_STATUS_SUCCESS) {
        result.successful = false;
        result.reason = "Failed to set Gamma, status = " + std::to_string(status);
      }
    } else if (param.get_name() == "flip_image") {
      flip_image_ = param.as_bool();
    } else {
      result.successful = false;
      result.reason = "Unknown parameter: " + param.get_name();
    }
  }
  return result;
}
}  // namespace mindvision_camera
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mindvision_camera::MVCameraNode)

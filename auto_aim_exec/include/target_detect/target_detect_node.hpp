#ifndef TARGET_DETECT_H
#define TARGET_DETECT_H

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

// Custom interfaces
#include "auto_aim_interface/msg/processed_image.hpp"
#include "auto_aim_interface/msg/object_det_output.hpp"
#include "auto_aim_interface/srv/object_detect.hpp"
#include "opencv2/opencv.hpp"
#include "cuda.h"
#include "cuda_runtime.h"
#include "NvInfer.h"
#include "nppi.h"
#include "nppi_geometry_transforms.h"
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/msg/string.hpp"

#include "target_detect/tools.hpp"
namespace target_detect
{
  class TargetDetectNode : public rclcpp::Node
  {
  public:
    explicit TargetDetectNode(rclcpp::NodeOptions const& options=rclcpp::NodeOptions().use_intra_process_comms(true));
    ~TargetDetectNode() override
    {
      RCLCPP_INFO(this->get_logger(), "Detect node destroyed!");
    }

  private:
    // static函数无法使用this指针，只能用实例化的对象，因此不方便使用
    // static void Sub_Callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
    //                          const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_info);
    void Pre_Process(int dst_height, int dst_width);
    void do_obj_inference();

    //image_transport::CameraSubscriber camera_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

    rclcpp::Publisher<auto_aim_interface::msg::ObjectDetOutput>::SharedPtr output_pub;
    auto_aim_interface::msg::ObjectDetOutput output_msg_temp;
    // auto_aim_interface::msg::ObjectDetOutput output_msg;
    nvinfer1::ICudaEngine *obj_engine;
    nvinfer1::IExecutionContext *obj_context;
    cudaStream_t obj_stream;
    int obj_input_idx, obj_output_idx, obj_input_width = 960, obj_input_height = 768;
    nvinfer1::Dims obj_input_dims, obj_output_dims;
    int obj_input_size, obj_output_size;
    void *obj_buffers[2];

    // preprocess needed variable
    const float32_t input_scale[3] = {0.00392157, 0.00392157, 0.00392157};
    const int DstOrder[3] = {2, 1, 0};
    uint8_t *ori_buf, *dev_ori_buf, *image_resize_buf;
    float32_t *image_float_buf;
    float32_t *input_buf;
    bool context_avaliable = true;

    int image_width = 960, image_height = 768;

    std::thread inference_thread;

    bool receive_flag = false;
    bool inference_flag = false;
    bool receive_protect = false;
    // mutexes
    std::mutex receive_mutex;
    std::mutex inference_mutex;
    std::mutex endinfer_mutex;
    std::mutex endrec_mutex;

    std::mutex resize_buf_mutex;
    std::mutex input_buf_mutex;
    std::mutex context_avaliable_mutex;

    int ori_buf_size, inter_buf_size;
    int frame_temp = 0;
  };
}

#endif
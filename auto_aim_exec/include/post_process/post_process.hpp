#ifndef POST_PROCESS_H
#define POST_PROCESS_H

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>

// Custom interfaces
#include <Eigen/Eigen>
#include "auto_aim_interface/msg/processed_image.hpp"
#include "auto_aim_interface/msg/object_det_output.hpp"
#include "auto_aim_interface/srv/object_detect.hpp"
#include "auto_aim_interface/msg/target_angle_output.hpp"
#include "auto_aim_interface/msg/imu_output.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"
#include "cuda.h"
#include "cuda_runtime.h"
#include "NvInfer.h"
#include "nppi.h"
#include "nppi_geometry_transforms.h"
#include "std_msgs/msg/string.hpp"
#include "time.h"
#include <sstream>
#include <deque>
#include <cstdio>
#include <string.h>
#include "fstream"
#include <vector>
#include "string"
#include "cmath"
#include "utils.hpp"
#include "yaml-cpp/yaml.h"
#include "kalman/predict_kalman.hpp"

#define image_width 960
#define image_height 768
#define PI 3.141592653
#define BIG_ARMOR_LENGTH 23   // unit: cm
#define BIG_ARMOR_WIDTH 5.8
#define SMALL_ARMOR_LENGTH 13.2
#define SMALL_ARMOR_WIDTH 5.4
#define gravity 9.8


namespace post_process
{
  class PostProcessNode : public rclcpp::Node
  {
  public:
    enum
    {
      red = 0,
      blue
    }; // 红蓝双方
    enum
    {
      small = 0,
      big
    }; // 大小装甲板
    explicit PostProcessNode(rclcpp::NodeOptions const &options = rclcpp::NodeOptions().use_intra_process_comms(true));
    ~PostProcessNode() override
    {
      RCLCPP_INFO(this->get_logger(), "Postprocess node destroyed!");
    }

  private:
    std::vector<std::string> target_label;          // 用于存储网络推理输出的标签
    std::vector<utils::pred_box> output_box;        // 存储findcontours函数的结果轮廓输出
    std::vector<utils::pred_box> temp_box;          // 存储经过过滤后的轮廓，例如大小过滤等
    std::vector<cv::Point3f> objP_small;            // 小装甲板世界坐标系坐标点值
    std::vector<cv::Point3f> objP_big;              // 大装甲板世界坐标系坐标点值
    std::vector<cv::Point2f> points;                // 2D平面像素坐标系下像素点坐标
    utils::pred_box last_box;                       // 记录上一时刻的box
    utils::pred_box result_box;                     // 需要做姿态解算并计算旋转角度的box
    cv::Mat cam;                                    // 相机内参矩阵
    cv::Mat dist;                                   // 相机畸变矩阵
    cv::Mat rvecs = cv::Mat::zeros(3, 1, CV_64FC1); // 旋转向量
    cv::Mat tvecs = cv::Mat::zeros(3, 1, CV_64FC1); // 平移向量
    cv::Mat rotM = cv::Mat::eye(3, 3, CV_64F);      // 旋转矩阵
    cv::Mat rotT = cv::Mat::eye(3, 3, CV_64F);      // 平移矩阵
    cv::Mat P;                                      // 深度矩阵
    bool catch_target = false;                      // 判断是否识别到目标，识别到与没识别到将会采用不同的选取目标策略

    double theta_x, theta_y, theta_z, distance;     // 最终解算角度与距离
    bool test = true;                               // 为true则显示一些调试画面

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_;                 // 创建一个接收节点，用于接收模型推理的输出结果
    rclcpp::Subscription<auto_aim_interface::msg::ObjectDetOutput>::SharedPtr sub_; 
    rclcpp::Subscription<auto_aim_interface::msg::ImuOutput>::SharedPtr imu_sub;

    int threshold = 70, maxVal = 255;              // 设置图像二值化时的阈值
    cv::Mat binary_img;                            // 用于存储提取掩模时的二进制图像载体
    std::vector<std::vector<cv::Point>> contours;  // 经过findcontours函数找到的轮廓的存储空间
    std::vector<cv::Point2f> target_point;         // 识别到的装甲板四边形四个角点像素坐标
    const int area_threshold = 100;                // 最小匹配区域面积

    auto_aim_interface::msg::TargetAngleOutput output_msg;      // 输出姿态解算后的角度信息
    rclcpp::Publisher<auto_aim_interface::msg::TargetAngleOutput>::SharedPtr output_pub;

    bool pixel = false;      // if true, use angle output, else use pixel output
    char target_color = red; // 目标颜色，红色red，蓝色blue
    int target_box_id = -1;  // 目标车辆id
    double fx, fy, cx, cy;   // 相机的两轴焦距以及光心对应像素点坐标

    int temp_frame = 0;           // use for debug

    Eigen::Vector3d pc;           // point axis for camera
    Eigen::Vector3d pw;           // point axis for world
    Eigen::Matrix3d R_CI;
    Eigen::Matrix3d F;

    double bullet_speed = 17;        // bullet shoot speed

    std::mutex imu_mutex;         // thread lock of imu data
    bool imu_mutex_var = false;
    float angle[3];               // eular angle
    enum{roll=0, yaw, pitch};
    float q[4];                   
    float acc[3];
    float gyro[3]; 
    Eigen::Quaternionf q_;
  private:                 // function
    void declareParameters(void);
    void camera_info_init(std::string file_path);
    void get_model_label(std::string file_path, std::vector<std::string> &labels);
    void armor_world_init(std::vector<cv::Point3f> &small, std::vector<cv::Point3f> &big);

    void handle_pred(float *data, float conf_thresh = 0.2, float nms_thresh = 0.01, int8_t big_cls_num = 9, int na = 0, int no = 0);

    void get_mask_img(std::vector<utils::pred_box> &result_box, int &id, const cv::Mat &raw_img, cv::Mat &to_img,
                      int binary_thresh, int maxval, double extend_w = 1.6, double extend_h = 1.2);
    bool is_light(cv::RotatedRect &rrect);
    void get_match_armor(std::vector<std::vector<cv::Point>> &_contours, const cv::Mat &raw_img, std::vector<cv::Point2f> &target_out,
                         int area_thresh = 20);
    void pub_angle_data(const std::vector<cv::Point2f> &target);
    double cal_gravity_compensation(double &bullet_speed, Eigen::Vector3d &_pc);

  };
}

#endif
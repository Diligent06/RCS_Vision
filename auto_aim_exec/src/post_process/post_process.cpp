#include "post_process/post_process.hpp"

namespace post_process
{
  PostProcessNode::PostProcessNode(rclcpp::NodeOptions const &options) : Node("post_process", options)
  {
    // PredictorKalman predictkalman();
    // 相机畸变矩阵与内参矩阵初始化
    std::string camera_info_path = PROJECT_DIR"/config/mindvision_camera/camera_info.yaml";
    camera_info_init(camera_info_path);
    // 装甲板坐标系初始化
    armor_world_init(objP_small, objP_big);
    // 读取标签名称
    std::string label_path = PROJECT_DIR"/model/robomaster.names";
    get_model_label(label_path, target_label);
    // 测试标签，true则显示图像
    test = true;
    // 如果为true，则计算像素点差值进行输出后处理数据
    pixel = false;
    // 输出节点
    this->output_pub = this->create_publisher<auto_aim_interface::msg::TargetAngleOutput>(
        "/target_angle_output", 10);
    // 初始化参数
    // this->declare_parameters();
    auto callback_ = [this](auto_aim_interface::msg::ObjectDetOutput::SharedPtr msg) -> void
    {
      // RCLCPP_INFO(this->get_logger(), "i am in callback");
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg->raw_image, sensor_msgs::image_encodings::TYPE_8UC3);
      //cv::imwrite("./rawimg" + std::to_string(temp_frame) + ".jpg", cv_ptr->image);
      //std::cout << cv_ptr->image.size() << std::endl;
      // RCLCPP_INFO(this->get_logger(), "nihao");
      // videowriter << cv_ptr->image;
      handle_pred(msg->output.data(), 0.2, 0.01, 9, msg->na, msg->no);
      if(test)
      {
        cv::imshow("raw_image", cv_ptr->image);
        cv::waitKey(1);
      }

      // //  如果没有识别到装甲板则返回
      if ((int)(output_box.size()) == 0)
      {
        //RCLCPP_INFO(this->get_logger(), "I can't find any armor region!");
        return;
      }
      target_box_id = -1;
      for (size_t i = 0; i < output_box.size(); i++)
      {
        if (output_box[i].bcls == target_color)
        {
          target_box_id = i;
          break;
        }
      }
      if (target_box_id == -1)
        return;
      // // 转为灰度图并扣出识别装甲板附近的区域做传统视觉处理
      cv::Mat src;
      this->get_mask_img(output_box, target_box_id, cv_ptr->image, src, threshold, maxVal);
      // if (test)
      // {
      //   cv::imshow("detect_img", src);
      //   cv::waitKey(1);
      // }
      this->get_match_armor(contours, src, target_point);
      if ((int)(target_point.size() != 4))
      {
        //RCLCPP_INFO(this->get_logger(), "Find %d armor but not 4!", (int)(target_point.size()));
        return;
      }
      // sort(target_point.begin(), target_point.end(), utils::target_sort);   // left top, right top, left buttom, right buttom
      if (test)
      {
        for (int i = 0; i < 1; i++)
        {
          for (int j = 0; j < 4; j++)
          {
            cv::line(cv_ptr->image, target_point[i * 4 + j], target_point[(j + 1) % 4 + i * 4], cv::Scalar(0, 255, 0), 4);
          }
        }
        cv::imshow("nihao.jpg", cv_ptr->image);
        cv::waitKey(1);
      }
      this->pub_angle_data(target_point);
    };
    sub_ = create_subscription<auto_aim_interface::msg::ObjectDetOutput>("object_det_output", 10, callback_);
    auto imu_callback = [this](auto_aim_interface::msg::ImuOutput::SharedPtr msg) -> void
    {
      std::lock_guard<std::mutex> imu_lock(imu_mutex);
      switch(msg->category)
      {
        case 1:
          angle[roll] = msg->angle_roll;
          angle[yaw] = msg->angle_yaw;
          angle[pitch] = msg->angle_pitch;
          //std::cout << "roll:" << angle[roll] << ",yaw:" << angle[yaw] << ",pitch:" << angle[pitch] << std::endl;
          break;
        case 2:
          cudaMemcpy(q, msg->q.data(), 4 * sizeof(float), cudaMemcpyHostToHost);
          //std::cout << "q:" << q[0] << ',' << q[1] << ',' << q[2] << ',' << q[3] << std::endl;
          break;
        case 3:
          cudaMemcpy(acc, msg->acc.data(), 3 * sizeof(float), cudaMemcpyHostToHost);
          cudaMemcpy(gyro, msg->gyro.data(), 3 * sizeof(float), cudaMemcpyHostToHost);
          //std::cout << "acc:" << acc[0] << ',' << acc[1] << ',' << acc[2] << std::endl;
          break;
        default:
          break;
      }
    };
    imu_sub = create_subscription<auto_aim_interface::msg::ImuOutput>("/imu_output", 10, imu_callback);
  }
  void PostProcessNode::handle_pred(float *data, float conf_thresh, float nms_thresh, int8_t big_cls_num, int na, int no)
  {
    float *head;
    float score;
    output_box.clear();
    temp_box.clear();
    int len = no - 5;
    for (int i = 0; i < na; i++)
    {
      head = data + i * no;
      score = utils::max(head + 5, len);
      score = pow(score, 0.4) * pow(head[4], 0.6);
      if (score < conf_thresh)
        continue;
      temp_box.emplace_back();
      auto &box = temp_box.back();
      box.cls = utils::argmax(head + 5, len);
      box.score = score;
      box.bcls = box.cls < big_cls_num;
      box._x = head[0];
      box._y = head[1];
      box.w = head[2];
      box.h = head[3];
      // 如果在追踪模式下，那么我们就需要计算识别区域装甲板之间的距离
      // if(catch_target == true){
      //   box.dist = pow((box._x - last_box._x), 2) + pow((box._y - last_box._y), 2);
      // }
    }
    sort(temp_box.begin(), temp_box.end(), utils::cmp);
    // 如果上一帧没有识别到目标，那么选取检测分数最高的作为击打目标
    // if (catch_target == false)
    //   sort(temp_box.begin(), temp_box.end(), utils::cmp);
    // else
    // {
    //   sort(temp_box.begin(), temp_box.end(), utils::track);
    // }
    // NMS结算
    bool keep = true;
    for (int i = 0; i < (int)(temp_box.size()); i++)
    {
      keep = true;
      for (int j = 0; j < (int)(output_box.size()); j++)
      {
        if (temp_box[i].bcls != output_box[j].bcls)
          continue;
        if (utils::calculate_iou(temp_box[i], output_box[j]) < nms_thresh)
          continue;
        keep = false;
      }
      if (keep)
        output_box.push_back(temp_box[i]);
    }
    // // 计算追踪装甲板位置
    // if(catch_target == true && (int)(output_box.size()) != 0){
    //     result_box = output_box[0];
    //     int delta_x = output_box[0]._x - last_box._x;
    //     int delta_y = output_box[0]._y - last_box._y;
    //     result_box._x = std::max(0, std::min(image_width, int(output_box[0]._x + sigma_x * delta_x)));
    //     result_box._y = std::max(0, std::min(image_height, int(output_box[0]._y + sigma_y * delta_y)));
    // }
    // // 如果检测到目标，则进入目标追踪，否则进入检测模式
    // if((int)(output_box.size()) == 0)
    //   catch_target = false;
    // else{
    //   last_box = output_box[0];
    //   catch_target = true;
    // }
  }
  void PostProcessNode::pub_angle_data(const std::vector<cv::Point2f> &target)
  {
    // 做pnp姿态解算，计算出深度距离信息
    if (output_box[target_box_id].cls == 1 || output_box[target_box_id].cls == 10)
      cv::solvePnP(objP_big, target, cam, dist, rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
    else
      cv::solvePnP(objP_small, target, cam, dist, rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
    cv::cv2eigen(tvecs, pc);             // x, y, z;     
    {
      std::lock_guard<std::mutex> imu_lock(imu_mutex);
      pw = predictkalman.predict(pc, q);
    }
    //std::cout << pc[0] << ' ' << pc[1] << ' ' << pc[2] << std::endl; // pc: yaw pitch distance
    // std::cout << "angle_pitch:" << std::atan2(pc[1], std::sqrt(pc[0]*pc[0]+pc[2]*pc[2])) << std::endl;
    //std::cout << pw[0] << ' ' << pw[1] << ' ' << pw[2] << std::endl;
    pc[1] = cal_gravity_compensation(bullet_speed, pc);
    output_msg.distance = pw.norm();
    output_msg.angle_yaw = std::atan2(pw[0], pw[1]) / PI * 180.f;
    output_msg.angle_pitch = std::atan2(pc[1], std::sqrt(pc[0]*pc[0]+pc[2]*pc[2])) * 180.f / PI;
    std::cout << "dis:" << output_msg.distance << ";yaw:" << output_msg.angle_yaw << ";pitch:" << output_msg.angle_pitch << std::endl; 
    // gravity compensation
    // float fly_time = pc.norm() / bullet_speed / 100.0f;
    // float new_y = pc[1] - 1 / 2.0 * gravity * fly_time * fly_time;
    // float new_pitch = std::atan2(new_y, std::sqrt(pc[2]*pc[2]+pc[0]+pc[0])) * 180.f / PI;
    // output_msg.angle_pitch = new_pitch;   // use gravity compensation
    // output_msg.angle_pitch = std::atan2(pc[1], std::sqrt(pc[2]*pc[2]+pc[0]*pc[0])) * 360.f / PI; // no use gravity compensation
    //std::cout << "distance:" << pc[2] << ";pitch:" << output_msg.angle_pitch << ";yaw:" << output_msg.angle_yaw << std::endl;
    output_pub->publish(output_msg);
  }
  void PostProcessNode::get_mask_img(std::vector<utils::pred_box> &result_box, int &id, const cv::Mat &raw_img, cv::Mat &to_img,
                                     int binary_thresh, int maxval, double extend_w, double extend_h)
  {
    cv::cvtColor(raw_img, binary_img, cv::COLOR_BGR2GRAY);
    cv::threshold(binary_img, binary_img, binary_thresh, maxval, cv::THRESH_BINARY);
    cv::Mat mask = cv::Mat::zeros(cv::Size(raw_img.cols, raw_img.rows), CV_8UC1);
    cv::rectangle(mask, cv::Rect(std::max(result_box[id]._x - result_box[id].w / 3 * extend_w, 0.0), std::max(result_box[id]._y - result_box[id].h / 2 * extend_h, 0.0), result_box[id].w * extend_w, result_box[id].h * extend_h),
                  cv::Scalar(255), -1);
    cv::Mat src;
    binary_img.copyTo(to_img, mask);
  }
  void PostProcessNode::get_match_armor(std::vector<std::vector<cv::Point>> &_contours, const cv::Mat &raw_img,
                                        std::vector<cv::Point2f> &target_out, int area_thresh)
  {
    _contours.clear();
    target_out.clear();
    cv::findContours(raw_img, _contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point> > temp_contours;
    for(size_t i = 0; i < _contours.size(); i++)
    {
      float area = cv::contourArea(_contours[i]);
      if(area < area_thresh)
        continue;
      cv::RotatedRect rrect = cv::minAreaRect(_contours[i]);
      if(!is_light(rrect))
        continue;
      temp_contours.push_back(_contours[i]);
    }
    bool compat_flag = false;
    for(size_t i = 0; i < temp_contours.size(); i++)
    {
      for(size_t j = i + 1; j < _contours.size(); j++)
      {
        cv::RotatedRect rrectA = cv::minAreaRect(_contours[i]);
        cv::RotatedRect rrectB = cv::minAreaRect(_contours[j]);
        float A_h, A_w, B_h, B_w;
        if (rrectA.size.height > rrectA.size.width)
        {
          A_h = rrectA.size.height;
          A_w = rrectA.size.width;
        }
        else
        {
          A_h = rrectA.size.width;
          A_w = rrectA.size.height;
        }
        if (rrectB.size.height > rrectB.size.width)
        {
          B_h = rrectB.size.height;
          B_w = rrectB.size.width;
        }
        else
        {
          B_h = rrectB.size.width;
          B_w = rrectB.size.height;
        }
        float armor_ratio = A_h < B_h ? A_h / B_h : B_h / A_h;

        float avg_armor_height = (A_h + B_h) / 2;
        distance = sqrt((rrectA.center.x - rrectB.center.x) * (rrectA.center.x - rrectB.center.x) +
                        (rrectA.center.y - rrectB.center.y) * (rrectA.center.y - rrectB.center.y));
        float center_distance = distance / avg_armor_height;
        bool center_distance_ok = (center_distance < 2.8 && center_distance > 0.8) ||
                                  (center_distance < 4.3 && center_distance > 3.2);

        float vertical_angle = std::abs(std::atan((rrectB.center.y - rrectA.center.y) /
                                                  (rrectB.center.x - rrectA.center.x))) / CV_PI * 180;
        bool angle_ok = vertical_angle < 45;
        if (!angle_ok || !center_distance_ok)
          continue;
        else
        {
          cv::Point2f *verticesA = new cv::Point2f[4];
          rrectA.points(verticesA);
          cv::Point2f *verticesB = new cv::Point2f[4];
          rrectB.points(verticesB);
          utils::rrect_sort(verticesA, 4);
          utils::rrect_sort(verticesB, 4);
          if(verticesA[0].x + verticesA[2].x < verticesB[0].x + verticesB[2].x)
          {
            target_out.push_back(cv::Point2f((int)((verticesA[0].x + verticesA[1].x) / 2), (int)((verticesA[0].y + verticesA[1].y) / 2)));
            target_out.push_back(cv::Point2f((int)((verticesB[0].x + verticesB[1].x) / 2), (int)((verticesB[0].y + verticesB[1].y) / 2)));
            target_out.push_back(cv::Point2f((int)((verticesA[2].x + verticesA[3].x) / 2), (int)((verticesA[2].y + verticesA[3].y) / 2)));
            target_out.push_back(cv::Point2f((int)((verticesB[2].x + verticesB[3].x) / 2), (int)((verticesB[2].y + verticesB[3].y) / 2)));
          }
          else
          {
            target_out.push_back(cv::Point2f((int)((verticesB[0].x + verticesB[1].x) / 2), (int)((verticesB[0].y + verticesB[1].y) / 2)));
            target_out.push_back(cv::Point2f((int)((verticesA[0].x + verticesA[1].x) / 2), (int)((verticesA[0].y + verticesA[1].y) / 2)));
            target_out.push_back(cv::Point2f((int)((verticesB[2].x + verticesB[3].x) / 2), (int)((verticesB[2].y + verticesB[3].y) / 2)));
            target_out.push_back(cv::Point2f((int)((verticesA[2].x + verticesA[3].x) / 2), (int)((verticesA[2].y + verticesA[3].y) / 2)));
          }
          compat_flag = true;
          break;
        }
      }
      if(compat_flag)
        break;
    }
    return;
  }
  bool PostProcessNode::is_light(cv::RotatedRect &rrect)
  {
    float ratio = rrect.size.width / rrect.size.height;
    ratio = rrect.size.width > rrect.size.height ? 1.0 / ratio : ratio;
    float angle = rrect.angle;
    float delta_angle = rrect.size.width > rrect.size.height ? std::fabs(90.0 - angle) : std::fabs(angle);
    // RCLCPP_INFO(this->get_logger(), "ratio:%f, angle:%f", ratio, delta_angle);
    return ratio > 0.1 && ratio < 0.55 && delta_angle < 33.0;
  }
  void PostProcessNode::declareParameters(void)
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;

    param_desc.description = "target armor color";
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 1;
    target_color = this->declare_parameter("target_color", 0, param_desc);

    param_desc.description = "show debug image flag";
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 1;
    test = this->declare_parameter("post_test", false, param_desc);
  }
  void PostProcessNode::camera_info_init(std::string file_path)
  {
    YAML::Node config = YAML::LoadFile(file_path);   // 载入文件
    std::vector<double> cam_mat = config["camera_matrix"]["data"].as<std::vector<double>>();
    int cam_mat_col = config["camera_matrix"]["cols"].as<int>();
    std::vector<double> dist_coef = config["distortion_coefficients"]["data"].as<std::vector<double>>();
    int dist_coef_col = config["distortion_coefficients"]["cols"].as<int>();
    cv::Mat cam_temp(cam_mat);
    cv::Mat dist_temp(dist_coef);
    cam = cam_temp.clone();
    dist = dist_temp.clone();
    int channel = 1;
    cam = cam.reshape(channel, cam_mat_col);
    dist = dist.reshape(channel, dist_coef_col);    // 变换矩阵形状
    cx = cam.at<double>(0, 2);
    cy = cam.at<double>(1, 2);
    fx = cam.at<double>(0, 0);
    fy = cam.at<double>(1, 1);
    return;
  }
  void PostProcessNode::get_model_label(std::string file_path, std::vector<std::string> &labels)
  {
    std::ifstream ReadFile;
    std::string tmp;
    ReadFile.open(file_path.c_str());
    if (ReadFile.fail())
      RCLCPP_INFO(this->get_logger(), "Read file failed!");
    while (getline(ReadFile, tmp, '\n'))
    {
      labels.push_back(tmp);
    }
    ReadFile.close();
  }
  void PostProcessNode::armor_world_init(std::vector<cv::Point3f> &small, std::vector<cv::Point3f> &big)
  {
    small.clear();
    small.push_back(cv::Point3f(SMALL_ARMOR_LENGTH/2, -SMALL_ARMOR_WIDTH/2, 0));  // 正对着装甲板，左上灯条角点，单位cm
    small.push_back(cv::Point3f(-SMALL_ARMOR_LENGTH/2, -SMALL_ARMOR_WIDTH/2, 0)); // 右上灯条角点
    small.push_back(cv::Point3f(SMALL_ARMOR_LENGTH/2, SMALL_ARMOR_WIDTH/2, 0));   // 左下灯条角点
    small.push_back(cv::Point3f(-SMALL_ARMOR_LENGTH/2, SMALL_ARMOR_WIDTH/2, 0));  // 右下灯条角点
    big.clear();
    big.push_back(cv::Point3f(BIG_ARMOR_LENGTH/2, -BIG_ARMOR_WIDTH/2, 0));  // 正对着装甲板，左上灯条角点，单位cm
    big.push_back(cv::Point3f(-BIG_ARMOR_LENGTH/2, -BIG_ARMOR_WIDTH/2, 0)); // 右上灯条角点
    big.push_back(cv::Point3f(BIG_ARMOR_LENGTH/2, BIG_ARMOR_WIDTH/2, 0));   // 左下灯条角点
    big.push_back(cv::Point3f(-BIG_ARMOR_LENGTH/2, BIG_ARMOR_WIDTH/2, 0));  // 右下灯条角点
  }
  double PostProcessNode::cal_gravity_compensation(double &bullet_speed, Eigen::Vector3d &_pc)
  {
    double target_high = _pc[1]/100.0;
    double theta = 0;
    double fly_time = 0;
    double delta_h = 0;
    for(int i = 0; i < 10; i++)
    {
      theta = std::atan(target_high / _pc[2]/100.0);
      fly_time = _pc[2] / 100.0 / (bullet_speed * std::cos(theta));
      delta_h = _pc[1] / 100.0 - (target_high - 1.0/2.0*gravity*fly_time*fly_time);
      target_high += delta_h;
    }
    //std::cout << "target_high:" << target_high * 100 << ";final_high:" << _pw[2] << std::endl;
    return target_high * 100;
  }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(post_process::PostProcessNode)

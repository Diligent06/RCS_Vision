#include "serial/Serial_node.hpp"

namespace serial
{
  SerialNode::SerialNode(rclcpp::NodeOptions const &options) : Node("serial", options)
  , robot_port("/dev/vision_serial", 115200), imu_port("/dev/imu_serial", 115200)//, robot_port("/dev/vision_serial", 115200)//
  {
    if (!imu_port.isOpen())
      imu_port.open();
    if (!robot_port.isOpen())
      robot_port.open();
    send_data_buf[0] = 0xef;
    auto callback_ = [this](auto_aim_interface::msg::TargetAngleOutput::SharedPtr msg) -> void
    {
      to_stm32_angle.angle_pitch.value = msg->angle_pitch;
      to_stm32_angle.angle_yaw.value = msg->angle_yaw;
      to_stm32_angle.distance.value = msg->distance;
      memcpy(send_data_buf+1, to_stm32_angle.angle_pitch.c, 4 * sizeof(uint8_t));
      memcpy(send_data_buf+5, to_stm32_angle.angle_yaw.c, 4 * sizeof(uint8_t));
      memcpy(send_data_buf+9, to_stm32_angle.distance.c, 4 * sizeof(uint8_t));
      // std::cout << send_data_buf[1] << ',' << send_data_buf[2] << ',' << send_data_buf[3] << ',' << send_data_buf[4] << std::endl;
      Append_CRC16_Check_Sum(send_data_buf, 15);
      if(robot_port.isOpen())
        robot_port.write(send_data_buf, 15);
      else
        robot_port.open();
    };
    sub_ = create_subscription<auto_aim_interface::msg::TargetAngleOutput>("target_angle_output", 10, callback_);
    RCLCPP_INFO(this->get_logger(), "Finished Serial Init!!!");
    // call back action every 8ms
    // if (imu_port.isOpen())
    // {
    //   timer = this->create_wall_timer(
    //       std::chrono::milliseconds(9), std::bind(&SerialNode::timer_callback, this));
    // }
    this->imu_pub = this->create_publisher<auto_aim_interface::msg::ImuOutput>(
        "/imu_output", 10);
    imu_serial_receive = std::thread([this]() {
      while(1)
      {
        IMU_PORT_CHECK;
        if(imu_port.read(rec_buf, 1) == 1)
        {
          if(rec_buf[0] == 0x55)
          {
            IMU_PORT_CHECK;
            if(imu_port.read(rec_buf+1, 1) == 1)
            {
              if(rec_buf[1] == 0x55)
              {
                IMU_PORT_CHECK;
                if(imu_port.read(rec_buf+2, 2) != 2)
                  continue;
                if(imu_port.read(rec_buf+4, rec_buf[3]+1) != rec_buf[3]+1)
                  continue;
                uint8_t sum = 0;
                switch (rec_buf[2])
                {
                case 0x01:
                  for (uint8_t i = 0; i < pos_len; i++)
                    sum += rec_buf[i];
                  if (sum == rec_buf[pos_len])
                  {
                    auto_aim_interface::msg::ImuOutput::UniquePtr imu_msg(new auto_aim_interface::msg::ImuOutput());
                    imu_msg->category = 1;
                    memcpy(imu_angle_buf, rec_buf + 4, rec_buf[3] * sizeof(uint8_t));
                    imu_msg->angle_roll = (float)((int16_t)(imu_angle_buf[1] << 8) | imu_angle_buf[0]) / 32768 * 180;
                    imu_msg->angle_pitch = (float)((int16_t)(imu_angle_buf[3] << 8) | imu_angle_buf[2]) / 32768 * 180;
                    imu_msg->angle_yaw = (float)((int16_t)(imu_angle_buf[5] << 8) | imu_angle_buf[4]) / 32768 * 180;
                    imu_pub->publish(std::move(imu_msg));
                  }
                  else
                    RCLCPP_INFO(this->get_logger(), "rec:%d, CRC16 check failed", rec_buf[2]);
                  break;
                case 0x02:
                  for (uint8_t i = 0; i < quat_len; i++)
                    sum += rec_buf[i];
                  if (sum == rec_buf[quat_len])
                  {
                    auto_aim_interface::msg::ImuOutput::UniquePtr imu_msg(new auto_aim_interface::msg::ImuOutput());
                    imu_msg->category = 2;
                    memcpy(quaternion_buf, rec_buf + 4, rec_buf[3] * sizeof(uint8_t));
                    float q_[4];
                    q_[0] = (float)((int16_t)(quaternion_buf[1] << 8) | quaternion_buf[0]) / 32768;
                    q_[1] = (float)((int16_t)(quaternion_buf[3] << 8) | quaternion_buf[2]) / 32768;
                    q_[2] = (float)((int16_t)(quaternion_buf[5] << 8) | quaternion_buf[4]) / 32768;
                    q_[3] = (float)((int16_t)(quaternion_buf[7] << 8) | quaternion_buf[6]) / 32768;
                    memcpy(imu_msg->q.data(), q_, 3 * sizeof(float));
                    imu_pub->publish(std::move(imu_msg));
                  }
                  else
                    RCLCPP_INFO(this->get_logger(), "rec:%d, CRC16 check failed", rec_buf[2]);
                  break;
                case 0x03:
                  for (uint8_t i = 0; i < gyro_len; i++)
                    sum += rec_buf[i];
                  if (sum == rec_buf[gyro_len])
                  {
                    auto_aim_interface::msg::ImuOutput::UniquePtr imu_msg(new auto_aim_interface::msg::ImuOutput());
                    imu_msg->category = 3;
                    memcpy(gyro_buf, rec_buf + 4, rec_buf[3] * sizeof(uint8_t));
                    float acc_[3];
                    float gyro_[3];
                    acc_[0] = (float)((int16_t)(gyro_buf[1] << 8) | gyro_buf[0]) / 32768 * ACC_FSR;
                    acc_[1] = (float)((int16_t)(gyro_buf[3] << 8) | gyro_buf[2]) / 32768 * ACC_FSR;
                    acc_[2] = (float)((int16_t)(gyro_buf[5] << 8) | gyro_buf[4]) / 32768 * ACC_FSR;
                    gyro_[0] = (float)((int16_t)(gyro_buf[7] << 8) | gyro_buf[6]) / 32768 * GYRO_FSR;
                    gyro_[1] = (float)((int16_t)(gyro_buf[9] << 8) | gyro_buf[8]) / 32768 * GYRO_FSR;
                    gyro_[2] = (float)((int16_t)(gyro_buf[11] << 8) | gyro_buf[10]) / 32768 * GYRO_FSR;
                    memcpy(imu_msg->acc.data(), acc_, 3 * sizeof(float));
                    memcpy(imu_msg->gyro.data(), gyro_, 3 * sizeof(float));
                    imu_pub->publish(std::move(imu_msg));
                  }
                  else
                  {
                    RCLCPP_INFO(this->get_logger(), "rec:%d, CRC16 check failed", rec_buf[2]);
                  }
                  break;
                default:
                  break;
                }
              }
            }
          }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
      }
    });
    imu_serial_receive.detach();
    // imu_serial_receive = std::thread([this](){
    //   while (1)
    //   {
    //     if(!imu_port.isOpen())
    //     {
    //       imu_port.open();
    //       continue;
    //     }
    //     if(imu_port.read(rec_buf, 1) == 1)
    //     {
    //       if(rec_buf[0] == 0x55)
    //       {
    //         if(imu_port.read(rec_buf+1, 1) == 1)
    //         {
    //           if(rec_buf[1] == 0x55)
    //           {
    //             if(imu_port.read(rec_buf+2, 2) != 2)
    //               continue;
    //             if(imu_port.read(rec_buf+4, rec_buf[3]+1) != rec_buf[3]+1)
    //               continue;
    //             uint8_t sum = 0;
    //             switch (rec_buf[2])
    //             {
    //             case 0x01:
    //               for (uint8_t i = 0; i < pos_len; i++)
    //                 sum += rec_buf[i];
    //               if (sum == rec_buf[pos_len])
    //               {
    //                 auto_aim_interface::msg::ImuOutput::UniquePtr imu_msg(new auto_aim_interface::msg::ImuOutput());
    //                 imu_msg->category = 1;
    //                 memcpy(imu_angle_buf, rec_buf + 4, rec_buf[3] * sizeof(uint8_t));
    //                 imu_msg->angle_roll = (float)((int16_t)(imu_angle_buf[1] << 8) | imu_angle_buf[0]) / 32768 * 180;
    //                 imu_msg->angle_pitch = (float)((int16_t)(imu_angle_buf[3] << 8) | imu_angle_buf[2]) / 32768 * 180;
    //                 imu_msg->angle_yaw = (float)((int16_t)(imu_angle_buf[5] << 8) | imu_angle_buf[4]) / 32768 * 180;
    //                 imu_pub->publish(std::move(imu_msg));
    //               }
    //               else
    //                 RCLCPP_INFO(this->get_logger(), "rec:%d, CRC16 check failed", rec_buf[2]);
    //               break;
    //             case 0x02:
    //               for (uint8_t i = 0; i < quat_len; i++)
    //                 sum += rec_buf[i];
    //               if (sum == rec_buf[quat_len])
    //               {
    //                 auto_aim_interface::msg::ImuOutput::UniquePtr imu_msg(new auto_aim_interface::msg::ImuOutput());
    //                 imu_msg->category = 2;
    //                 memcpy(quaternion_buf, rec_buf + 4, rec_buf[3] * sizeof(uint8_t));
    //                 float q_[4];
    //                 q_[0] = (float)((int16_t)(quaternion_buf[1] << 8) | quaternion_buf[0]) / 32768;
    //                 q_[1] = (float)((int16_t)(quaternion_buf[3] << 8) | quaternion_buf[2]) / 32768;
    //                 q_[2] = (float)((int16_t)(quaternion_buf[5] << 8) | quaternion_buf[4]) / 32768;
    //                 q_[3] = (float)((int16_t)(quaternion_buf[7] << 8) | quaternion_buf[6]) / 32768;
    //                 memcpy(imu_msg->q.data(), q_, 3 * sizeof(float));
    //                 imu_pub->publish(std::move(imu_msg));
    //               }
    //               else
    //                 RCLCPP_INFO(this->get_logger(), "rec:%d, CRC16 check failed", rec_buf[2]);
    //               break;
    //             case 0x03:
    //               for (uint8_t i = 0; i < gyro_len; i++)
    //                 sum += rec_buf[i];
    //               if (sum == rec_buf[gyro_len])
    //               {
    //                 auto_aim_interface::msg::ImuOutput::UniquePtr imu_msg(new auto_aim_interface::msg::ImuOutput());
    //                 imu_msg->category = 3;
    //                 memcpy(gyro_buf, rec_buf + 4, rec_buf[3] * sizeof(uint8_t));
    //                 float acc_[3];
    //                 float gyro_[3];
    //                 acc_[0] = (float)((int16_t)(gyro_buf[1] << 8) | gyro_buf[0]) / 32768 * ACC_FSR;
    //                 acc_[1] = (float)((int16_t)(gyro_buf[3] << 8) | gyro_buf[2]) / 32768 * ACC_FSR;
    //                 acc_[2] = (float)((int16_t)(gyro_buf[5] << 8) | gyro_buf[4]) / 32768 * ACC_FSR;
    //                 gyro_[0] = (float)((int16_t)(gyro_buf[7] << 8) | gyro_buf[6]) / 32768 * GYRO_FSR;
    //                 gyro_[1] = (float)((int16_t)(gyro_buf[9] << 8) | gyro_buf[8]) / 32768 * GYRO_FSR;
    //                 gyro_[2] = (float)((int16_t)(gyro_buf[11] << 8) | gyro_buf[10]) / 32768 * GYRO_FSR;
    //                 memcpy(imu_msg->acc.data(), acc_, 3 * sizeof(float));
    //                 memcpy(imu_msg->gyro.data(), gyro_, 3 * sizeof(float));
    //                 imu_pub->publish(std::move(imu_msg));
    //               }
    //               else
    //               {
    //                 RCLCPP_INFO(this->get_logger(), "rec:%d, CRC16 check failed", rec_buf[2]);
    //               }
    //               break;
    //             default:
    //               break;
    //             }
    //           }
    //         }
    //       }
    //     }
    //     std::this_thread::sleep_for(std::chrono::microseconds(5000));
    //   }
    // });
    // imu_serial_receive.detach();
  }
  // void SerialNode::timer_callback()
  // {
  //   imu_rec_flag = false;
  //   while (1)
  //   {
  //     if (imu_port.read(rec_buf, 4) != 4)
  //       break;
  //     else
  //     {
  //       if (rec_buf[0] == 0x55 && rec_buf[1] == 0x55)
  //       {
  //         uint8_t sum = 0;
  //         imu_port.read(rec_buf + 4, rec_buf[3] + 1);
  //         switch (rec_buf[2])
  //         {
  //         case 0x01:
  //           for (uint8_t i = 0; i < pos_len; i++)
  //             sum += rec_buf[i];
  //           if (sum == rec_buf[pos_len])
  //           {
  //             memcpy(imu_angle_buf, rec_buf + 4, rec_buf[3] * sizeof(uint8_t));
  //             posture_angle.roll = (float)((int16_t)(imu_angle_buf[1] << 8) | imu_angle_buf[0]) / 32768 * 180;
  //             posture_angle.pitch = (float)((int16_t)(imu_angle_buf[3] << 8) | imu_angle_buf[2]) / 32768 * 180;
  //             posture_angle.yaw = (float)((int16_t)(imu_angle_buf[5] << 8) | imu_angle_buf[4]) / 32768 * 180;
  //           }
  //           imu_rec_flag = true;
  //           break;
  //         case 0x02:
  //           for (uint8_t i = 0; i < quat_len; i++)
  //             sum += rec_buf[i];
  //           if (sum == rec_buf[quat_len])
  //             memcpy(quaternion_buf, rec_buf + 4, rec_buf[3] * sizeof(uint8_t));
  //           //RCLCPP_INFO(this->get_logger(), "read buf id %d", rec_buf[2]);
  //           break;
  //         case 0x03:
  //           for (uint8_t i = 0; i < gyro_len; i++)
  //             sum += rec_buf[i];
  //           if (sum == rec_buf[gyro_len])
  //             memcpy(gyro_buf, rec_buf + 4, rec_buf[3] * sizeof(uint8_t));
  //           //RCLCPP_INFO(this->get_logger(), "read buf id %d", rec_buf[2]);
  //           break;
  //         default:
  //           break;
  //         }
  //       }
  //     }
  //   }
  //   if (!imu_rec_flag)
  //     return;
  //   memcpy(send_data_buf + imu_head, imu_angle_buf, imu_len * sizeof(uint8_t));
  //   {
  //     std::lock_guard<std::mutex> posture_lk(posture_mutex);
  //     posture_copy = postprocess_angle;
  //     if(detect_update){
  //       detect_update_copy = detect_update;
  //       detect_update = false;
  //     }
  //   }

  //   memcpy(send_data_buf + pitch_head, posture_copy.angle_pitch.c, posture_len * sizeof(uint8_t));
  //   memcpy(send_data_buf + yaw_head, posture_copy.angle_yaw.c, posture_len * sizeof(uint8_t));
  //   memcpy(send_data_buf + dis_head, posture_copy.distance.c, posture_len * sizeof(uint8_t));
  //   //RCLCPP_INFO(this->get_logger(), "detect_copy%f", detect_update_copy);
  //   if (detect_update_copy)
  //   {
  //     // 如果有新的识别帧过来，再运行重力补偿函数
  //     compensat.value = gravity_compensation(posture_copy, posture_angle.pitch);
  //     compensat.value = -compensat.value / pi * 180.f;
  //     memcpy(send_data_buf + compensat_head, compensat.c, posture_len * sizeof(uint8_t));
  //     // 将识别帧标志位置0，等待下一识别帧的到达
  //     detect_update_copy = false;
  //     send_data_buf[update_head] = 1;
  //     if(test++ % 1 == 0){
  //        RCLCPP_INFO(this->get_logger(), "compensation:%f,angle_pitch:%f,imu:%f", compensat.value, posture_copy.angle_pitch.value, posture_angle.pitch);
  //     }
  //   }
  //   else
  //   {
  //     send_data_buf[update_head] = 0;
  //   }
    
  //   Append_CRC16_Check_Sum(send_data_buf, total_len);

  //   if (robot_port.write(send_data_buf, total_len) == total_len)
  //   {
  //     //RCLCPP_INFO(this->get_logger(), "pitch:%f", posture_copy.angle_pitch.value);
  //     //RCLCPP_INFO(this->get_logger(), "pitch:%d %d %d %d", posture_copy.angle_pitch.c[0], posture_copy.angle_pitch.c[1], posture_copy.angle_pitch.c[2], posture_copy.angle_pitch.c[3]);
  //     return;
  //   }
  //   else
  //     RCLCPP_INFO(this->get_logger(), "send data unsuccessfully!");
  //   return;
  // }
  float SerialNode::gravity_compensation(Protocol::Target_angle &post_angle, float &imu_pitch)
  {
    // post_angle.angle_pitch < 0, 头偏下，应该往上转
    float theta1 = imu_pitch / 180.0 * pi;
    theta1 += post_angle.angle_pitch.value / 180.0 * pi;
    float s = post_angle.distance.value / 100.0 * cos(theta1);
    float d = post_angle.distance.value / 100.0;
    float theta2;
    //RCLCPP_INFO(this->get_logger(), "theta1:%f,imu_pitch:%f", theta1, imu_pitch / 180.0 * pi);
    dichotomy(theta1, theta2, s, bullet_speed, d, dichotomy_epoch);
    //RCLCPP_INFO(this->get_logger(), "theta1:%f,theta2:%f", theta1, theta2);
    // if(test++ % 20 == 0){
    //   std::cout << "imu:" << theta1 << ",pitch:" << post_angle.angle_pitch.value << std::endl;
    //   std::cout << "s:" << s << ",d:" << d << std::endl;
    //   std::cout << "theta2=" << theta2 << std::endl;
    // }
    return theta1 + theta2;
  }
  void SerialNode::dichotomy(float &theta1, float &theta2, float &s, float &v, float &d, int &epoch)
  {
    float left = -1, right = 1;
    float ds = s + d_h * sin(theta1);
    for (int i = 0; i < epoch; i++)
    {
      theta2 = (left + right) / 2;
      if (ds * sin(theta2) / cos(theta2) - 1 / 2.0 * gravity * ds * ds / (v * v * cos(theta2) * cos(theta2)) + d * sin(theta1) - d_h * cos(theta1) > 0)
        right = theta2;
      else
        left = theta2;
    }
    theta2 = left;
    return;
  }
  void SerialNode::Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
  {
    uint16_t wCRC = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
      return;
    }
    wCRC = Get_CRC16_Check_Sum((uint8_t *)pchMessage, dwLength - 2, CRC_INIT);
    pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff);
    pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
    return;
  }
  uint16_t SerialNode::Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
  {
    uint8_t chData;
    if (pchMessage == NULL)
    {
      return 0xFFFF;
    }
    while (dwLength--)
    {
      chData = *pchMessage++;
      (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    }
    return wCRC;
  }
  bool SerialNode::Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
  {
    uint16_t wExpected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
      return false;
    }
    wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) ==
                                                                  pchMessage[dwLength - 1]);
  }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(serial::SerialNode)

#ifndef PROTOCOL_HPP
#define PROTOCOL_HPP
namespace Protocol
{
  // vision 
  typedef union{
    float value;
    unsigned char c[4];
  }float2uchar;
  typedef struct
  {
    float2uchar angle_pitch;
    float2uchar angle_yaw;
    //float2uchar angle_raw;
    float2uchar distance;
  }Target_angle;
  // serial
  typedef struct
  {
    float roll;
    float pitch;
    float yaw;
    float distance;
  }imu_posture_data;
}
#endif
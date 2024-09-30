#include <serial/serial.h>
#include <ros/ros.h>

namespace icm42688
{

#define GET_ACCEL_GYRO_TEMP 0xA0
#define GET_ACCEL 0xA1
#define GET_GYRO 0xA2
#define GET_TEMP 0xA3
#define GET_BIAS 0xA4
#define GET_STORED_BIAS 0xA5
#define GET_ADAPTED_BIAS 0xA6
#define SET_BIAS 0xB0
#define REPLACE_SPECIFIED_BIAS 0xB1
#define ADAPTED_SPECIFIED_BIAS 0xB2
#define RESTART_IMU 0xC0
#define CHECK_FIRMWARE 0xD0

class ICM42688ROS
{
public:
  ICM42688ROS();
  ~ICM42688ROS();
  void run();

private:
  ros::NodeHandle nh_;
  ros::Publisher imu_pub_;
  serial::Serial serial_;
};
}  // namespace icm42688

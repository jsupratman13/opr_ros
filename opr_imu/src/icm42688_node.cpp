#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <cstdint>
#include <thread>
#include "opr_imu/icm42688_node.hpp"
#include "opr_imu/crc_table.hpp"

namespace icm42688
{
ICM42688ROS::ICM42688ROS()
{
  ros::NodeHandle private_nh("~");

  std::string port;
  int baudrate;
  private_nh.getParam("port", port);
  private_nh.getParam("baudrate", baudrate);

  serial::Timeout timeout = serial::Timeout::simpleTimeout(10);
  serial_.setTimeout(timeout);
  serial_.setPort(port);
  serial_.setBaudrate(baudrate);
  serial_.setParity(serial::parity_none);
  serial_.setStopbits(serial::stopbits_one);
  try
  {
    serial_.open();
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("Unable to open port " << port);
    throw e;
  }

  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 1);
}

ICM42688ROS::~ICM42688ROS()
{
  serial_.close();
}

void ICM42688ROS::run()
{
  while (ros::ok())
  {
    // send command
    std::vector<uint8_t> header = { 0xFE, 0xFE };
    uint8_t send_data_length = 6;
    uint8_t command = GET_ACCEL_GYRO_TEMP;
    std::vector<uint8_t> send_data;
    send_data.push_back(header[0]);
    send_data.push_back(header[1]);
    send_data.push_back(command);
    send_data.push_back(send_data_length);

    uint16_t crc = getCRC16(send_data);
    send_data.push_back(crc & 0xFF);
    send_data.push_back((crc >> 8) & 0xFF);

    serial_.write(send_data);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // receive data
    uint8_t buffer[4];
    // check if 4 bytes are received, header + command + length
    if (serial_.read(buffer, 4) != 4)
      continue;
    // check if header is correct
    if (!std::equal(header.begin(), header.end(), buffer))
      continue;
    // get rest of the data
    uint8_t recv_data_length = buffer[3];
    std::vector<uint8_t> data;
    serial_.read(data, recv_data_length);
    std::vector<uint8_t> recv_data;
    recv_data.insert(recv_data.end(), buffer, buffer + 4);
    recv_data.insert(recv_data.end(), data.begin(), data.end());
    if (!getCRC16(recv_data))
      continue;

    // parse data
    std::vector<float> accel_gyro_temp;
    for (size_t i = 5; i < recv_data.size() - 2; i += 4)
    {
      float value = *reinterpret_cast<float*>(&recv_data[i]);
      accel_gyro_temp.push_back(value);
    }
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "imu";
    imu_msg.linear_acceleration.x = accel_gyro_temp[0];
    imu_msg.linear_acceleration.y = accel_gyro_temp[1];
    imu_msg.linear_acceleration.z = accel_gyro_temp[2];
    imu_msg.angular_velocity.x = accel_gyro_temp[3];
    imu_msg.angular_velocity.y = accel_gyro_temp[4];
    imu_msg.angular_velocity.z = accel_gyro_temp[5];
    imu_pub_.publish(imu_msg);

    ros::spinOnce();
  }
}
}  // namespace icm42688

int main(int argc, char** argv)
{
  ros::init(argc, argv, "icm42688_node");
  icm42688::ICM42688ROS icm42688_ros;
  icm42688_ros.run();
  return 0;
}

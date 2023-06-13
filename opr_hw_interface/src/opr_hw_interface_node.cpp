/*********************************************************************
 * Copyright (c) 2023 Joshua Supratman
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ********************************************************************/

#include <combined_robot_hw/combined_robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "opr_hw_interface_node");
  ros::NodeHandle nh, nh_private("~");

  combined_robot_hw::CombinedRobotHW hw;
  if (hw.init(nh, nh_private))
  {
    ROS_FATAL_STREAM("Initializing hardware interfaces failed");
    ros::shutdown();
    return 1;
  }
  ROS_INFO_STREAM("Initializing hardware interfaces succeeded, starting hardware interface");

  controller_manager::ControllerManager cm(&hw, nh);
  ros::AsyncSpinner spinner(1);
  ros::Rate rate(100);  // 100Hz update rate

  spinner.start();
  while (ros::ok())
  {
    hw.read(ros::Time::now(), rate.expectedCycleTime());
    cm.update(ros::Time::now(), rate.expectedCycleTime());
    hw.write(ros::Time::now(), rate.expectedCycleTime());
    rate.sleep();
  }
  spinner.stop();
  return 0;
}

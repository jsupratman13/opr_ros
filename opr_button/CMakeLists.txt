cmake_minimum_required(VERSION 3.0.2)
project(opr_button)

find_package(
  catkin REQUIRED
  COMPONENTS rospy
             std_msgs
             serial
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    serial
)

add_custom_target(${PROJECT_NAME}_install_python_package ALL COMMAND python -m pip install platformio)

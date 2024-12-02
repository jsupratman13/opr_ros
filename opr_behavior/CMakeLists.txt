cmake_minimum_required(VERSION 3.0.2)
project(opr_behavior)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy)

file(
  GLOB programs
  RELATIVE ${PROJECT_SOURCE_DIR}/scripts
  scripts/*.py
)

install(
  PROGRAMS ${programs}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

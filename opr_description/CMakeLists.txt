cmake_minimum_required(VERSION 3.0.2)
project(opr_description)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY meshes
            launch
            robots
            rviz
            xacro
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

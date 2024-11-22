cmake_minimum_required(VERSION 3.0.2)
project(opr_gazebo)

find_package(catkin REQUIRED)
catkin_package()

install(
  DIRECTORY materials
            models
            worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

cmake_minimum_required(VERSION 3.0.2)
project(opr_imu)

add_compile_options(-std=c++17)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             sensor_msgs
             std_msgs
             tf
             serial
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    std_msgs
    tf
    serial
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/icm42688_node.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

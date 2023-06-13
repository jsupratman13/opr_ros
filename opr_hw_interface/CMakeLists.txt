cmake_minimum_required(VERSION 3.0.2)
project(opr_hw_interface)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             controller_manager
             hardware_interface
             combined_robot_hw
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    controller_manager
    hardware_interface
    combined_robot_hw
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/opr_hw_interface_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

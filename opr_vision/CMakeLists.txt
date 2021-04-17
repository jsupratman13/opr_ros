cmake_minimum_required(VERSION 2.8.3)
project(opr_vision)

add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS "-std=c++1z -fPIC")
set(CMAKE_CXX_STANDARD 17)


add_definitions(-DGPU)
add_definitions(-D_CRT_SECURE_NO_WARNINGS)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    roscpp
    sensor_msgs
    std_msgs
    opr_msgs
)

catkin_package()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt5 COMPONENTS Core Gui REQUIRED)

set(QT_LIBRARIES Qt5::Gui)
#add_definitions(-DQT_NO_KEYWORDS)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
	${BOOST_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
)


set(vision_src
	src/vision.cpp
	src/vision.h
	src/object_types.h
	src/detector/object_detector.cpp
	src/detector/object_detector.h
	src/detector/convert_color.cpp
	src/detector/convert_color.h
	src/detector/detector_tool.cpp
	src/detector/detector_tool.h
	src/detector/detector_color_table.cpp
	src/detector/detector_color_table.h
	src/detector/color_table.cpp
	src/detector/color_table.h
	src/detector/particle.h
	src/detector/ball_particle.cpp
	src/detector/ball_particle.h
)


set(vision_links
	${OpenCV_LIBS}
	zbar
	stdc++fs
)

add_executable(vision ${vision_src})
target_link_libraries(vision
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${QT_LIBRARIES}
	${vision_links}
)



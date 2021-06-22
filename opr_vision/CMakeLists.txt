cmake_minimum_required(VERSION 3.1)
project(opr_vision)
# project(vision)
# add_subdirectory("detector/pytorch_wl")

set(CMAKE_CXX_FLAGS "-std=c++14 -fPIC")

# set(ICE_DIR $ENV{ICE_HOME} CACHE PATH "ZeroC ICE installation dir(Default: $ICE_HOME)")

# set(USE_VREP_SIMULATOR CACHE BOOL FORCE)
# if($ENV{VREP_DIR})
#     set(USE_VREP_SIMULATOR True)
# endif($ENV{VREP_DIR})

# set(USE_WEBOTS_VHSC CACHE BOOL FORCE)
# set(USE_WEBOTS_DIRECT CACHE BOOL FORCE)

# if(USE_VREP_SIMULATOR)
#     add_definitions(-DVREP_SIMULATOR)
# elseif(USE_WEBOTS_VHSC)
#     add_definitions(-DWEBOTS_VHSC)
# elseif(USE_WEBOTS_DIRECT)
#     add_definitions(-DWEBOTS_DIRECT)
#     set(USE_VREP_SIMULATOR False)
#     add_definitions(-DWEBOTS_SIMULATOR)
    find_package(Boost REQUIRED COMPONENTS system date_time)
#     find_package(Protobuf REQUIRED)
#     protobuf_generate_cpp(PIC_PROTO_SOURCES PIC_PROTO_HEADERS picture.proto)
# endif(USE_VREP_SIMULATOR)

# set(USE_GPU CACHE BOOL FORCE)

# if(USE_GPU)
    add_definitions(-DGPU)
# endif(USE_GPU)

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

find_package(OpenCV)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  opr_msgs
)
catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
	# ${ICE_DIR}/include
	# ${PROJECT_BINARY_DIR}
	# ${CMAKE_BINARY_DIR}/gen
	# ${CMAKE_BINARY_DIR}/webots
	# ${CMAKE_CURRENT_SOURCE_DIR}
	${BOOST_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
	# ../SimDriver
	# ../HCServer
	# ../HPL
	# ../webots
	# ${CMAKE_BINARY_DIR}/webots
	# ${TORCH_INCLUDE_DIRS}
	# ./detector/pytorch_wl/
)

# link_directories(
#     ${ICE_DIR}/lib
# )

# if(USE_GPU)
#     include_directories(
#         /usr/local/cuda/include
#         $ENV{HOME}/darknet/include
#     )
#     link_directories(
#         $ENV{HOME}/darknet
#     )
# endif(USE_GPU)

set(vision_src
    src/main.cpp
    src/vision.cpp
    src/vision.h
    src/image_grabber/image_grabber.cpp
    src/image_grabber/image_grabber.h
    src/detector/object_detector.cpp
    src/detector/object_detector.h
    src/object_types.h
    src/detector/convert_color.cpp
    src/detector/convert_color.h
    src/detector/detector_tool.cpp
    src/detector/detector_tool.h
    src/detector/color_table.cpp
    src/detector/color_table.h
    src/detector/particle.h
    src/detector/ball_particle.cpp
    src/detector/ball_particle.h
	# ${PIC_PROTO_HEADERS}
	# ${PIC_PROTO_SOURCES}
)

# if(USE_VREP_SIMULATOR)
#     set(vision_src
#         ${vision_src}
#         image_grabber/image_grabber_vrep.cpp
#         image_grabber/image_grabber_vrep.h
#         image_grabber/image_grabber_file.cpp
#         image_grabber/image_grabber_file.h
#     )
# elseif(USE_WEBOTS_DIRECT)
#     set(vision_src
#         ${vision_src}
#         image_grabber/image_grabber_webots_direct.cpp
#         image_grabber/image_grabber_webots_direct.h
#         image_grabber/image_grabber_file.cpp
#         image_grabber/image_grabber_file.h
#     )
# elseif(USE_WEBOTS_VHSC)
#     set(vision_src
#         ${vision_src}
#         image_grabber/image_grabber_webots.cpp
#         image_grabber/image_grabber_webots.h
#         image_grabber/image_grabber_file.cpp
#         image_grabber/image_grabber_file.h
#     )
# else(USE_VREP_SIMULATOR)
#     set(vision_src
#         ${vision_src}
#         image_grabber/image_grabber_camera.cpp
#         image_grabber/image_grabber_camera.h
#         image_grabber/image_grabber_v4l.cpp
#         image_grabber/image_grabber_v4l.h
#         image_grabber/v4l/v4l2capture.cpp
#         image_grabber/v4l/v4l2capture.h
#         image_grabber/v4l/video_adjust.h
#         image_grabber/v4l/video_adjust_KBCRM05VU.cpp
#         image_grabber/v4l/video_adjust_KBCRM05VU.h
#         image_grabber/v4l/video_adjust_KBCRS02MU.cpp
#         image_grabber/v4l/video_adjust_KBCRS02MU.h
#     )
# endif(USE_VREP_SIMULATOR)
	set(vision_src
		${vision_src}
		src/image_grabber/image_grabber_ros.cpp
		src/image_grabber/image_grabber_ros.h
		src/image_grabber/image_grabber_file.cpp
		src/image_grabber/image_grabber_file.h
	)

# if(USE_GPU)
#     set(vision_src
#         ${vision_src}
#         detector/detector_yolo.cpp
#         detector/detector_yolo.h
#         detector/detector_fcn.cpp
#         detector/detector_fcn.h
#         detector/detector_wl.cpp
#         detector/detector_wl.h
#         detector/predict_by_deep_learning.cpp
#         detector/predict_by_deep_learning.h
#     )
# else(USE_GPU)
	set(vision_src
		${vision_src}
		src/detector/detector_color_table.cpp
		src/detector/detector_color_table.h
	)
# endif(USE_GPU)

# add_library(vision
#     ${vision_src}
# )

set(vision_links
	${OpenCV_LIBS}
	zbar
	stdc++fs
	# wldetector
)
# if(USE_VREP_SIMULATOR)
#     set(vision_links ${vision_links}
#         SimIPCLib
#     )
# elseif(USE_WEBOTS_VHSC)
#     find_package(Protobuf REQUIRED)
#     set(vision_links ${vision_links}
#         ${PROTOBUF_LIBRARIES}
#         WebotsClientLib
#         rt
#     )
# elseif(USE_WEBOTS_DIRECT)
#     set(vision_links ${vision_links}
#         ${PROTOBUF_LIBRARIES}
#         rt
#         )
# endif(USE_VREP_SIMULATOR)

# if (USE_GPU)
#     set(vision_links ${vision_links}
#         darknet
#     )
# endif(USE_GPU)

add_executable(vision ${vision_src}) #pass
target_link_libraries(vision
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRAREIS}
	# PRIVATE
	${vision_links}
)

# set_property(TARGET vision PROPERTY CXX_STANDARD 14)

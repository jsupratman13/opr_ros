#!/usr/bin/env pipenv-shebang
# -*- coding:utf-8 -*-

# Copyright (c) 2024 Joshua Supratman
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from ultralytics import YOLO
from vision_msgs.msg import Detection2D
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose


class BallDetection(object):
    def __init__(self) -> None:
        self.__model_name = rospy.get_param('~model_name', 'yolov8m.pt')
        self.__score_threshold = rospy.get_param('~score_thresh', 0.3)
        self.__iou_threshold = rospy.get_param('~iou_thresh', 0.7)
        self.__max_detection = rospy.get_param('~max_detection', 2)
        self.__classes = rospy.get_param('~classes', None)
        self.__ball_label = rospy.get_param('~ball_label', 'sports ball')
        self.__ball_diameter = rospy.get_param('~ball_diameter', 0.13)
        self.__pan_joint = rospy.get_param('~pan_joint', 'head_yaw_joint')
        self.__rate = rospy.Rate(rospy.get_param('~rate', 10))

        self.__model = YOLO(self.__model_name)
        self.__model.fuse()

        self.__cv_bridge = CvBridge()
        self.__result_pub = rospy.Publisher('detections', Detection2DArray, queue_size=1)
        self.__image_sub = rospy.Subscriber('image_rect', Image, self.__image_cb)
        self.__debug_pub = rospy.Publisher('debug', Image, queue_size=1)

        self.__kalman = cv2.KalmanFilter(4, 2)  # 4 states (x, y, dx, dy), 2 measurements (x, y)
        self.__kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                                    [0, 1, 0, 0]], np.float32)
        self.__kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                                   [0, 1, 0, 1],
                                                   [0, 0, 1, 0],
                                                   [0, 0, 0, 1]], np.float32)
        self.__kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03  # Process noise
        self.__last_ball_detect = None

        self.__ball_x_center = None
        self.__ball_distance = None
        self.__ball_detected = None
        self.__pan_pub = rospy.Publisher('position_controller/command', Float64, queue_size=1)

        camera_info = rospy.wait_for_message('camera_info', CameraInfo, 1.0)
        fx = camera_info.K[0]
        self.__focal_length = fx
        img_width = camera_info.width
        self.__hfov = 2 * np.arctan(img_width / (2 * fx))
        self.__img_x_center = img_width / 2.0

        rospy.loginfo('BallDetection')

    def __image_cb(self, img_msg: Image) -> None:
        try:
            cv_image = self.__cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f'Error converting ROS image to openCV: {e}')
            return

        results = self.__model.predict(
            source=cv_image,
            conf=self.__score_threshold,
            iou=self.__iou_threshold,
            verbose=False,
            max_det=self.__max_detection,
            classes=self.__classes)

        ball_detect = False
        detection_msg = Detection2DArray()
        detection_msg.header = img_msg.header
        for result in results[0].boxes:
            x_min, y_min, x_max, y_max = map(int, result.xyxy[0].tolist())
            cls = result.cls[0].item()
            confidence = result.conf[0].item()

            detection = Detection2D()
            detection.bbox.center.x = (x_min + x_max) / 2.0
            detection.bbox.center.y = (y_min + y_max) / 2.0
            detection.bbox.size_x = x_max - x_min
            detection.bbox.size_y = y_max - y_min

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = int(cls)
            hypothesis.score = confidence
            detection.results.append(hypothesis)

            detection_msg.detections.append(detection)

            label = self.__model.names[int(cls)]
            if label == self.__ball_label:
                ball_detect = True
                self.__ball_x_center = (x_min + x_max) / 2.0
                distance = (self.__ball_diameter * self.__focal_length) / (y_max - y_min)
                self.__ball_distance = np.sqrt(distance**2 + 0.42**2)

        self.__result_pub.publish(detection_msg)

        if ball_detect:
            measurement = np.array([[self.__ball_x_center], [0]], np.float32)
            self.__kalman.correct(measurement)
            self.__last_ball_detect = rospy.Time.now()
            self.__ball_detected = True

        debug_img = cv_image.copy()
        for result in results:
            debug_img = result.plot(img=debug_img)
        try:
            debug_msg = self.__cv_bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
        except Exception as e:
            rospy.logerr(f'Error converting openCV to ROS image: {e}')
            return
        debug_msg.header = img_msg.header
        self.__debug_pub.publish(debug_msg)

    def update(self) -> None:
        self.__rate.sleep()

        try:
            joint_states = rospy.wait_for_message('joint_states', JointState, 1.0)
            index = joint_states.name.index(self.__pan_joint)
            current_pan = joint_states.position[index]
        except Exception:
            rospy.logwarn(f'Failed to get current {self.__pan_joint} position')
            return

        predicted_x = self.__ball_x_center
        # if self.__last_ball_detect is None or (rospy.Time.now() - self.__last_ball_detect) > rospy.Duration(1.0):
        #     prediction = self.__kalman.predict()
        #     predicted_x = prediction[0][0]
        if self.__ball_detected and predicted_x:
            ball_x_diff = (predicted_x - self.__img_x_center) * (self.__ball_distance / self.__focal_length)
            offset = np.arctan2(ball_x_diff, self.__ball_distance)
            print(f'ball_x: {ball_x_diff}')
            print(f'distance: {self.__ball_distance}')
            print(f'true_offset: {offset}')
            print('-----')
            if abs(offset) > 0.05:
                target_pan = current_pan - offset
                self.__pan_pub.publish(Float64(target_pan))
            self.__ball_detected = False


if __name__ == '__main__':
    rospy.init_node('ball_detection')
    node = BallDetection()
    try:
        while not rospy.is_shutdown():
            node.update()
    except rospy.ROSInterruptException:
        pass

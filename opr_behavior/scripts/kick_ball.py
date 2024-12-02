#!/usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np
import rospy
import tf2_ros
from actionlib import SimpleActionClient
from hajime_walk_msgs.msg import HajimeMotionAction
from hajime_walk_msgs.msg import HajimeMotionGoal
from hajime_walk_msgs.msg import HajimeWalk
from std_msgs.msg import Empty
from std_msgs.msg import Float64


class KickBallOnly(object):
    def __init__(self) -> None:
        self.motion = SimpleActionClient('hajime_walk/motion', HajimeMotionAction)
        self.motion.wait_for_server(rospy.Duration(1.0))
        self.walk = rospy.Publisher('hajime_walk/walk', HajimeWalk, queue_size=1)
        self.cancel = rospy.Publisher('hajime_walk/cancel', Empty, queue_size=1)
        self.head_pan = rospy.Publisher('head_yaw_controller/command', Float64, queue_size=1)
        self.mid_stride_x = rospy.get_param('~mid_stride_x', 10)
        self.max_stride_y = rospy.get_param('~max_stride_y', 20)
        self.mid_stride_y = rospy.get_param('~mid_stride_y', 10)
        self.max_stride_th = rospy.get_param('~max_stride_th', 10)
        self.period = rospy.get_param('~walk_period', 0)

        self.tf2_buffer = tf2_ros.Buffer(rospy.Duration(3))
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

        self.rate = rospy.Rate(10)
        rospy.loginfo('Ready')

    def stop(self) -> None:
        self.cancel.publish(Empty())

    def run(self) -> None:
        try:
            ball_pos = self.tf2_buffer.lookup_transform('ball', 'base_link', rospy.Time(0)).transform
            ball_x = ball_pos.translation.y
            ball_y = ball_pos.translation.x
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(str(e))
            self.cancel.publish(Empty())
            return

        ball_deg = np.rad2deg(np.arctan2(ball_y, ball_x))
        if np.fabs(ball_deg) > 30:
            walk_th = np.clip(ball_deg, -self.max_stride_th, self.max_stride_th)
            self.walk.publish(HajimeWalk(0, self.period, 0, 0, int(walk_th)))
        elif np.fabs(ball_y) > 0.5:
            walk_y = np.clip(ball_y, -self.max_stride_y, self.max_stride_y)
            self.walk.publish(HajimeWalk(0, self.period, 0, int(walk_y), 0))
        elif np.fabs(ball_x) > 0.1:
            walk_x = np.clip(ball_x * 100, -self.mid_stride_x, self.mid_stride_x)
            walk_y = 0  # np.clip(ball_y * 10, -self.mid_stride_y, self.mid_stride_y)
            self.walk.publish(HajimeWalk(0, self.period, int(walk_x), int(walk_y), 0))
        else:
            print('kick!')
            self.cancel.publish(Empty())
            rospy.sleep(1.0)
            self.motion.send_goal(HajimeMotionGoal(motion_id=33))
            self.motion.wait_for_result()
        print('---')
        print(f'ball_x: {ball_x}')
        print(f'ball_y: {ball_y}')
        print(f'ball_deg: {ball_deg}')


if __name__ == '__main__':
    rospy.init_node('kick_ball')
    rate = rospy.Rate(10)
    behavior = KickBallOnly()
    rospy.on_shutdown(behavior.stop)
    try:
        while not rospy.is_shutdown():
            behavior.run()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

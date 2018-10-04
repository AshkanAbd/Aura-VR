#!/usr/bin/env python

import math
<<<<<<< HEAD
import rospy
import numpy as np
import nav_msgs.msg
import tf.transformations


def get_map(map):
    global main_map
    h = map.info.height
    w = map.info.width
    new_map = np.asarray(map.data)
    reshape = new_map.reshape(h, w)
    main_map = map
    print(convert_from_robot_to_map(0,-2))


def get_odom(odom):
    q = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w
    )
    print((tf.transformations.euler_from_quaternion(q)[2]) * 180 / math.pi)


def convert_from_robot_to_map(robot_y, robot_x):
    global main_map
    map_x = (robot_x - main_map.info.origin.position.x) // main_map.info.resolution
    map_y = (robot_y - main_map.info.origin.position.y) // main_map.info.resolution
    return map_y, map_x

=======
import cv2 as cv
import numpy as np
import rospy
import time
import geometry_msgs.msg
>>>>>>> d8e967595b87e6c2f8621474fec2393c79a9bfb0


def publishing(msg):
    global pub, r
    i = 0
    while not rospy.is_shutdown() and i < 2:
        pub.publish(msg)
        i += 1
        r.sleep()


if __name__ == '__main__':
    rospy.init_node('move')
    pub = rospy.Publisher('/robot0/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    twist = geometry_msgs.msg.Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 1
    r = rospy.Rate(1)
    publishing(twist)
    time.sleep(9.28)  # ~2.32 for 90 degree
    twist.angular.z = 0
    publishing(twist)
    print('published')

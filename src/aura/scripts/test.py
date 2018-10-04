#!/usr/bin/env python

import cv2 as cv
import numpy as np
import rospy
import time
import geometry_msgs.msg


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

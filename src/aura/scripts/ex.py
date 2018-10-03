#!/usr/bin/env python

import numpy as np
import cv2 as cv
import rospy
import cv_bridge
import geometry_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg
import std_msgs.msg


def get_normal_image(img):
    global bridge
    frame = bridge.imgmsg_to_cv2(img, 'bgr8')
    cv.imshow('a' , frame)
    cv.waitKey(1)

def main():
    global bridge, victim_info_pub
    bridge = cv_bridge.CvBridge()
    rospy.init_node('hot_victim_detector' + namespace)
    rospy.Subscriber('/' + namespace + '/camera_ros/image', sensor_msgs.msg.Image, get_normal_image)
    victim_info_pub = rospy.Publisher('/' + namespace + '/victims/hot', std_msgs.msg.Float64MultiArray, queue_size=10)
    rospy.spin()


victim_info_pub = None
bridge = None
namespace = 'robot0'
if __name__ == '__main__':
    main()

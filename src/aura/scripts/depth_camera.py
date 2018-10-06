#!/usr/bin/env python2

import numpy as np
import sensor_msgs.msg
import sensor_msgs
import image_geometry.cameramodels
import image_geometry
import rospy
import cv2 as cv
import cv_bridge
import std_msgs
import std_msgs.msg


def get_depth_image(sensor_img):
    global bridge
    try:
        frame = bridge.imgmsg_to_cv2(sensor_img, "passthrough")
        img = np.array(frame, np.float64)
        nor_img = cv.normalize(img, img, 0, 1, cv.NORM_MINMAX)
        cv.imshow('f1', nor_img)
        cv.waitKey(1)
    except cv_bridge.CvBridgeError as e:
        pass


if __name__ == '__main__':
    namespace = 'robot0'
    rospy.init_node('depth_test')
    bridge = cv_bridge.CvBridge()
    rospy.Subscriber('/' + namespace + '/camera_depth/depth/image_raw', sensor_msgs.msg.Image, get_depth_image)
    rospy.spin()

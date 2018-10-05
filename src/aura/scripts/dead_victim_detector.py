#!/usr/bin/env python2.7

import rospy
import sensor_msgs.msg
import cv_bridge
import numpy as np
import cv2 as cv
import std_msgs.msg


def get_image(img):
    global bridge, victim_info_pub
    frame = bridge.imgmsg_to_cv2(img, 'bgr8')
    frame = cv.medianBlur(frame, 5)
    frame_hsv1 = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask1 = cv.inRange(frame_hsv1, lower_dead_color1, upper_dead_color1)
    frame = cv.bitwise_not(frame)
    frame_hsv2 = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask2 = cv.inRange(frame_hsv2, lower_dead_color2, upper_dead_color2)
    final_frame = cv.bitwise_and(frame, frame, mask=mask1)
    final_frame = cv.bitwise_and(final_frame, final_frame, mask=mask2)
    frame_edge = cv.Laplacian(final_frame, -1)
    frame_edge = cv.cvtColor(frame_edge, cv.COLOR_BGR2GRAY)
    _, contours, _ = cv.findContours(frame_edge, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    cnts_area = {}
    cv.waitKey(1)
    cv.imshow('dead_final', final_frame)
    if len(contours) == 0: return
    for cnt in contours:
        cnts_area[cv.contourArea(cnt)] = cnt
    areas = [i for i in cnts_area.keys()]
    areas.sort(reverse=True)
    main_cnt = cnts_area[areas[0]]
    x, y, w, h = cv.boundingRect(main_cnt)
    final_frame = cv.rectangle(final_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
    a = std_msgs.msg.Float64MultiArray()
    a.data = [x, y, w, h]
    victim_info_pub.publish(a)
    cv.imshow('dead_final', final_frame)


namespace = 'robot0'
lower_dead_color1 = np.array([[[0, 0, 0]]])
upper_dead_color1 = np.array([[[0, 50, 50]]])
lower_dead_color2 = np.array([[[0, 0, 100]]])
upper_dead_color2 = np.array([[[0, 50, 255]]])
bridge = None
victim_info_pub = None


def main():
    global bridge, victim_info_pub
    bridge = cv_bridge.CvBridge()
    rospy.init_node('dead_victim_detector_' + namespace)
    rospy.Subscriber('/' + namespace + '/camera_depth/rgb/image', sensor_msgs.msg.Image, get_image)
    victim_info_pub = rospy.Publisher('/' + namespace + '/victims/dead', std_msgs.msg.Float64MultiArray, queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    main()

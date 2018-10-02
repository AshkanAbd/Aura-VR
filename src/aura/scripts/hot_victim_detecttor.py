#!/usr/bin/env python2.7

import rospy
import cv2 as cv
import cv_bridge
import sensor_msgs.msg
import std_msgs.msg
import numpy as np


def get_normal_image(image):
    global bridge, lower_hot_color, upper_hot_color, normal_img
    frame = bridge.imgmsg_to_cv2(image, "bgr8")
    frame = cv.bilateralFilter(frame, 9, 75, 75)
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    _, frame_thresh = cv.threshold(frame_gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
    frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(frame_hsv, lower_hot_color, upper_hot_color)
    final_frame = frame.copy()
    final_frame = cv.bitwise_and(final_frame, final_frame, mask=frame_thresh)
    # end!!!
    # send to process img
    normal_img = final_frame


def get_thermal_image(image):
    global bridge, thermal_img
    frame = bridge.imgmsg_to_cv2(image, "bgr8")
    thermal_img = frame
    frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    frame = cv.bilateralFilter(frame, 9, 75, 75)
    _, frame_thresh1 = cv.threshold(frame, 180, 255, cv.THRESH_BINARY)
    _, frame_thresh = cv.threshold(frame_thresh1, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
    # end!!!
    # send to process img
    thermal_img = frame_thresh


def process_img():
    global normal_img, thermal_img, victim_info_pub
    while True:
        if thermal_img is None or normal_img is None:
            continue
        try:
            normal_gray = cv.cvtColor(normal_img, cv.COLOR_BGR2GRAY)
            _, normal_thresh = cv.threshold(normal_gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
            thermal_img1 = cv.resize(thermal_img, None, fx=2, fy=2)
            final_frame = cv.bitwise_and(normal_thresh, thermal_img1)
            frame_edge = cv.Laplacian(final_frame, -1)
            _, contours, _ = cv.findContours(frame_edge, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            cnts_area = {}
            cv.waitKey(1)
            cv.imshow('hot_final', final_frame)
            if len(contours) == 0:
                continue
            for cnt in contours:
                cnts_area[cv.contourArea(cnt)] = cnt
            areas = [i for i in cnts_area.keys()]
            areas.sort(reverse=True)
            main_cnt = cnts_area[areas[0]]
            x, y, w, h = cv.boundingRect(main_cnt)
            a = std_msgs.msg.Float64MultiArray()
            a.data = [x, y, w, h]
            final_frame = cv.rectangle(final_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            victim_info_pub.publish(a)
            cv.imshow('hot_final', final_frame)
        except Exception as e:
            del e


namespace = 'robot0'
lower_hot_color = np.array([[[0, 30, 30]]])
upper_hot_color = np.array([[[20, 255, 255]]])
normal_img = None
thermal_img = None
bridge = None
victim_info_pub = None


def main():
    global bridge, victim_info_pub
    bridge = cv_bridge.CvBridge()
    rospy.init_node('hot_victim_detector_' + namespace)
    rospy.Subscriber('/' + namespace + '/camera/thermal/image_raw', sensor_msgs.msg.Image, get_thermal_image)
    rospy.Subscriber('/' + namespace + '/camera_ros/image', sensor_msgs.msg.Image, get_normal_image)
    victim_info_pub = rospy.Publisher('/' + namespace + '/victims/hot', std_msgs.msg.Float64MultiArray, queue_size=10)
    process_img()
    rospy.spin()


if __name__ == '__main__':
    main()

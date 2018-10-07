#!/usr/bin/env python2

import numpy as np
import sensor_msgs.msg
import sensor_msgs
import rospy
import nav_msgs.msg
# import image_geometry.cameramodels
# import image_geometry
# import cv2 as cv
# import cv_bridge
import std_msgs
import std_msgs.msg


def get_depth_image(sensor_img: sensor_msgs.msg.CameraInfo):
    # global bridge
    # try:
    #     frame = bridge.imgmsg_to_cv2(sensor_img, "passthrough")
    #     img = np.array(frame, np.float64)
    #     nor_img = cv.normalize(img, img, 0, 1, cv.NORM_MINMAX)
    #     cv.imshow('f1', nor_img)
    #     cv.waitKey(1)
    # except cv_bridge.CvBridgeError as e:
    #     pass
    global a
    a = np.asarray(sensor_img.P).reshape(3, 4)


def convert_from_robot_to_map(robot_y, robot_x):
    global map_info
    map_x = (robot_x - map_info.info.origin.position.x) // map_info.info.resolution
    map_y = (robot_y - map_info.info.origin.position.y) // map_info.info.resolution
    return map_y, map_x


def get_victim(array: std_msgs.msg.Float64MultiArray):
    global a
    screen_x = (array.data[0])
    screen_y = (array.data[1])
    world_x = (screen_x - a[0, 2]) / a[0, 0]
    world_y = (screen_y - a[1, 2]) / a[1, 1]
    print(a)
    print(array.data)
    print(convert_from_robot_to_map(world_y, world_x))


a = None

if __name__ == '__main__':
    namespace = 'robot0'
    rospy.init_node('depth_test')
    # bridge = cv_bridge.CvBridge()
    map_info = (rospy.wait_for_message("/core/map", nav_msgs.msg.OccupancyGrid))
    rospy.Subscriber('/' + namespace + '/camera_depth/rgb/camera_depth_info', sensor_msgs.msg.CameraInfo,
                     get_depth_image)

    rospy.Subscriber('/' + namespace + '/victims/hot', std_msgs.msg.Float64MultiArray, get_victim)
    rospy.spin()

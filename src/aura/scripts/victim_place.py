#!/usr/bin/env python2

import numpy as np
import sensor_msgs.msg
import sensor_msgs
import rospy
import nav_msgs.msg
import image_geometry.cameramodels
import image_geometry
import cv2 as cv
import cv_bridge
import std_msgs
import std_msgs.msg
import tf.broadcaster
import tf.listener
import tf.msg
import tf.transformations
import image_geometry.cameramodels
import geometry_msgs.msg


def convert_from_map_to_robot(map_y, map_x):
    global map_info
    robot_x = (map_x * map_info.info.resolution) + map_info.info.origin.position.x
    robot_y = (map_y * map_info.info.resolution) + map_info.info.origin.position.y
    return robot_y, robot_x


def convert_from_robot_to_map(robot_y, robot_x):
    global map_info
    map_x = (robot_x - map_info.info.origin.position.x) // map_info.info.resolution
    map_y = (robot_y - map_info.info.origin.position.y) // map_info.info.resolution
    return map_y, map_x


# codes : 1) hot victim  , 2) dead victim
def get_victim(array):
    # global img_geo, const_hot_victim_x, odom_info, const_hot_victim_y, mark_publisher
    u = array.data[0] + (array.data[2] // 2)
    v = array.data[1] + array.data[3] - 3
    time = rospy.Time(0)
    listener = tf.listener.TransformListener()
    camera_point = img_geo.projectPixelTo3dRay((img_geo.rectifyPoint((u, v))))
    print(camera_point)
    point_msg.pose.position.x = camera_point[0] * 10
    point_msg.pose.position.y = camera_point[1] * 10
    point_msg.pose.position.z = 0
    point_msg.pose.orientation.x = 0
    point_msg.pose.orientation.y = 0
    point_msg.pose.orientation.z = 0
    point_msg.pose.orientation.w = 1
    point_msg.header.frame_id = img_geo.tfFrame()
    point_msg.header.stamp = time
    try:
        listener.waitForTransform(img_geo.tfFrame(), 'map', time, rospy.Duration(1))
        tf_point = listener.transformPose('map', point_msg)
        print(tf_point.pose.position.x, tf_point.pose.position.y)
        # print(convert_from_robot_to_map(tf_point.pose.position.y, tf_point.pose.position.x))
        print("###########################")
    except Exception:
        pass


def in_range(victim, x, y):
    if victim[0] - 2 <= x <= victim[0] + 2:
        return True
    if victim[1] - 2 <= y <= victim[1] + 2:
        return True
    return False


def get_odom(odom):
    global odom_info
    odom_info = odom


def check_victim_list(x, y):
    global victim_list
    for victim in victim_list:
        if in_range(victim, x, y):
            return False
    victim_list.append((x, y))
    return True


const_hot_victim_x = 6
const_hot_victim_y = 15
camera_info = sensor_msgs.msg.CameraInfo()
img_geo = image_geometry.PinholeCameraModel()
victim_list = []
odom_info = nav_msgs.msg.Odometry()
point_msg = geometry_msgs.msg.PoseStamped()

if __name__ == '__main__':
    namespace = 'robot0'
    rospy.init_node('victim_place')
    mark_publisher = rospy.Publisher('/core/mark_place', std_msgs.msg.Float64MultiArray, queue_size=10)
    map_info = (rospy.wait_for_message("/core/map", nav_msgs.msg.OccupancyGrid))
    camera_info = rospy.wait_for_message('/' + namespace + '/camera_depth/rgb/camera_depth_info',
                                         sensor_msgs.msg.CameraInfo)
    odom_info = rospy.wait_for_message('/' + namespace + '/odom', nav_msgs.msg.Odometry)
    img_geo.fromCameraInfo(camera_info)
    rospy.Subscriber('/' + namespace + '/odom', nav_msgs.msg.Odometry, get_odom)
    rospy.Subscriber('/' + namespace + '/victims/hot', std_msgs.msg.Float64MultiArray, get_victim)
    rospy.spin()

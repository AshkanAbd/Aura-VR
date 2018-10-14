#!/usr/bin/env python

import numpy as np
import rospy
import geometry_msgs.msg
import nav_msgs.msg
import std_msgs.msg
import tf.transformations
import math
import cv2 as cv


def get_info(core_map):
    global main_map
    main_map = core_map
    arr_map = np.asarray(core_map.data).reshape(core_map.info.width, core_map.info.height)
    normalize(arr_map)
    # q = (
    #     odom.pose.pose.orientation.x,
    #     odom.pose.pose.orientation.y,
    #     odom.pose.pose.orientation.z,
    #     odom.pose.pose.orientation.w
    # )
    # robot_yaw = tf.transformations.euler_from_quaternion(q)
    # robot_poses = convert_from_robot_to_map(odom.pose.pose.position.y, odom.pose.pose.position.x)
    # print(robot_yaw[2] * 180 / math.pi)
    # print(core_map.info)


def normalize(arr_map):
    arr_map[arr_map == 0] = 200
    arr_map[arr_map == 100] = 0
    arr_map[arr_map == -1] = 100
    cv.imwrite('/home/ashkan/Desktop/example.png', arr_map)


def convert_from_robot_to_map(robot_y, robot_x):
    global main_map
    map_x = (robot_x - main_map.info.origin.position.x) // main_map.info.resolution
    map_y = (robot_y - main_map.info.origin.position.y) // main_map.info.resolution
    return map_y, map_x


def convert_from_map_to_robot(map_y, map_x):
    global main_map
    robot_x = (map_x * main_map.info.resolution) + main_map.info.origin.position.x
    robot_y = (map_y * main_map.info.resolution) + main_map.info.origin.position.y
    return robot_y, robot_x


def main():
    rospy.init_node('find_victim_' + namespace)
    get_info(rospy.wait_for_message('/robot0/map', nav_msgs.msg.OccupancyGrid))


# codes : 1) hot victim  , 2) dead victim
# def get_victim(array):
#     # global img_geo, const_hot_victim_x, odom_info, const_hot_victim_y, mark_publisher
#     u = array.data[0] + (array.data[2] // 2)
#     v = array.data[1] + array.data[3] - 3
#     time = rospy.Time(0)
#     listener = tf.listener.TransformListener()
#     camera_point = img_geo.projectPixelTo3dRay((img_geo.rectifyPoint((u, v))))
#     print(camera_point)
#     point_msg.pose.position.x = camera_point[0] * 10
#     point_msg.pose.position.y = camera_point[1] * 10
#     point_msg.pose.position.z = 0
#     point_msg.pose.orientation.x = 0
#     point_msg.pose.orientation.y = 0
#     point_msg.pose.orientation.z = 0
#     point_msg.pose.orientation.w = 1
#     point_msg.header.frame_id = img_geo.tfFrame()
#     point_msg.header.stamp = time
#     try:
#         listener.waitForTransform(img_geo.tfFrame(), 'map', time, rospy.Duration(1))
#         tf_point = listener.transformPose('map', point_msg)
#         print(tf_point.pose.position.x, tf_point.pose.position.y)
#         # print(convert_from_robot_to_map(tf_point.pose.position.y, tf_point.pose.position.x))
#         print("###########################")
#     except Exception:
#         pass

main_map = None
namespace = 'robot0'
if __name__ == '__main__':
    main()

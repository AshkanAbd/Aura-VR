#!/usr/bin/env python3.5

import math
import rospy
import random
import numpy as np
import nav_msgs.msg
import std_msgs.msg
import actionlib.msg
import geometry_msgs.msg
import actionlib
import move_base_msgs.msg
import move_base.cfg
import base_local_planner.msg
import base_local_planner.cfg
import trajectory_msgs.msg
import actionlib_msgs.msg
import costmap_2d.msg
import local_planner_limits
import tf.transformations
import hot_victim_detecttor


def get_map(map):
    global map_info
    h = map.info.height
    w = map.info.width
    new_map = np.asarray(map.data)
    reshape = new_map.reshape(h, w)
    map_info = map


def get_odom(odom):
    q = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w
    )
    print((tf.transformations.euler_from_quaternion(q)[2]) * 180 / math.pi)


def convert_from_robot_to_map(robot_y, robot_x):
    global map_info
    map_x = (robot_x - map_info.info.origin.position.x) // map_info.info.resolution
    map_y = (robot_y - map_info.info.origin.position.y) // map_info.info.resolution
    return map_y, map_x


def convert_from_map_to_robot(map_y, map_x):
    global map_info
    robot_x = ((map_x) * map_info.info.resolution) + map_info.info.origin.position.x
    robot_y = ((map_y) * map_info.info.resolution) + map_info.info.origin.position.y
    return robot_y, robot_x


map_info = None

if __name__ == '__main__':
    hot_victim_detecttor.main()
    # rospy.init_node('a')
    # rospy.Subscriber('/robot0/odom', nav_msgs.msg.Odometry, get_odom)
    # rospy.Subscriber('/robot0/map', nav_msgs.msg.OccupancyGrid, get_map)
    # rospy.spin()

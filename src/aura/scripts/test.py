#!/usr/bin/env python

import math
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


def convert_from_map_to_robot(map_y, map_x):
    global main_map
    robot_x = ((map_x) * main_map.info.resolution) + main_map.info.origin.position.x
    robot_y = ((map_y) * main_map.info.resolution) + main_map.info.origin.position.y
    return robot_y, robot_x


main_map = None

if __name__ == '__main__':
    # rospy.Subscriber('/robot0/odom', nav_msgs.msg.Odometry, get_odom)
    rospy.init_node('a')
    rospy.Subscriber('/core', nav_msgs.msg.OccupancyGrid, get_map)
    rospy.spin()

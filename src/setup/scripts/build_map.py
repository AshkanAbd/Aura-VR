#!/usr/bin/env python3.5

import rospy
import nav_msgs.msg
import numpy as np
import sys

name_space = "aura1"
map_header = None
map_info = None
map_data = list


def map_cutter(robot_map: nav_msgs.msg.OccupancyGrid.data, weight: int = 50, height: int = 50):
    global map_data, map_info
    width = map_info.width
    height1 = map_info.height
    _build_map = np.array(robot_map).reshape(width, height1)
    _build_map = _build_map[weight:width - weight, height:height1 - height]
    x = width - (2 * weight)
    y = height1 - (2 * height)
    _build_map = _build_map.reshape(1, y * x)
    map_data = _build_map[0].tolist()


def map_listener(data: nav_msgs.msg.OccupancyGrid):
    global map_header, map_info
    map_header = data.header
    map_info = data.info
    map_cutter(data.data)


def listen_map():
    global name_space
    ns_map = rospy.Subscriber("/" + name_space + "/map", nav_msgs.msg.OccupancyGrid, map_listener)
    rospy.spin()


def send_map():
    global name_space, map_info, map_header, map_data
    map_publisher = rospy.Publisher("/" + name_space + "/build_map", nav_msgs.msg.OccupancyGrid, queue_size=10)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        new_map = nav_msgs.msg.OccupancyGrid()
        new_map.info = map_info
        new_map.header = map_header
        new_map.data = map_data
        map_publisher.publish(new_map)
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node('map_builder', anonymous=True)
    name_space = sys.argv[1]
    name_space = rospy.get_param("namespace", sys.argv[1])
    listen_map()
    send_map()

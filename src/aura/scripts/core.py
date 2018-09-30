#!/usr/bin/env python3.5

import rospy
import nav_msgs.msg
import numpy as np
import sys


def get_map(robot_map: nav_msgs.msg.OccupancyGrid, robot: str):
    global map_info
    map_info = robot_map
    build_core_map(robot_map)


def build_core_map(robot_map: nav_msgs.msg.OccupancyGrid):
    global core_map
    reshaped_map = np.asarray(robot_map.data).reshape(robot_map.info.height, robot_map.info.width)
    if core_map.shape != reshaped_map.shape:
        core_map = reshaped_map.copy()
    map1 = reshaped_map.copy()
    map1[map1 == 0] = -1
    core_map[map1 == 100] = 100
    map1 = reshaped_map.copy()
    map1[map1 == 100] = -1
    core_map[map1 == 0] = 0


def publish_core():
    global core_publisher, core_map, map_info
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        data_map = map_info
        data_map.data = core_map.reshape(map_info.info.width * map_info.info.height)
        core_publisher.publish(data_map)
        rate.sleep()


core_map = np.array([])
map_info = nav_msgs.msg.OccupancyGrid()

if __name__ == '__main__':
    robot0 = sys.argv[1]
    robot1 = sys.argv[2]
    robot2 = sys.argv[3]
    robot3 = sys.argv[4]
    rospy.init_node('core')
    rospy.Subscriber('/' + robot0 + '/map', nav_msgs.msg.OccupancyGrid, get_map, robot0)
    rospy.Subscriber('/' + robot1 + '/map', nav_msgs.msg.OccupancyGrid, get_map, robot1)
    rospy.Subscriber('/' + robot2 + '/map', nav_msgs.msg.OccupancyGrid, get_map, robot2)
    rospy.Subscriber('/' + robot3 + '/map', nav_msgs.msg.OccupancyGrid, get_map, robot3)
    core_publisher = rospy.Publisher('/core', nav_msgs.msg.OccupancyGrid, queue_size=20)
    publish_core()
    rospy.spin()

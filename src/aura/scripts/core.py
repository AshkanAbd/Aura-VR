#!/usr/bin/env python3

import rospy
import nav_msgs.msg
import numpy as np
import sys


class CoreMapBuilder:
    robot0 = 'robot0'
    robot1 = 'robot1'
    robot2 = 'robot2'
    robot3 = 'robot3'
    core_map = np.array([])
    main_map = nav_msgs.msg.OccupancyGrid()
    core_publisher = None
    rate = None

    def __init__(self, robot0='robot0', robot1='robot1', robot2='robot2', robot3='robot3'):
        self.robot1 = robot0
        self.robot2 = robot1
        self.robot3 = robot2
        self.robot0 = robot3
        rospy.init_node('core_builder')
        rospy.Subscriber('/' + self.robot1 + '/map', nav_msgs.msg.OccupancyGrid, self.get_robots_map)
        rospy.Subscriber('/' + self.robot2 + '/map', nav_msgs.msg.OccupancyGrid, self.get_robots_map)
        rospy.Subscriber('/' + self.robot3 + '/map', nav_msgs.msg.OccupancyGrid, self.get_robots_map)
        rospy.Subscriber('/' + self.robot0 + '/map', nav_msgs.msg.OccupancyGrid, self.get_robots_map)
        self.core_publisher = rospy.Publisher('/core/map', nav_msgs.msg.OccupancyGrid, queue_size=100)
        self.rate = rospy.Rate(10)
        self.publish_to_core()

    def get_robots_map(self, robot_map: nav_msgs.msg.OccupancyGrid):
        global map_info
        self.main_map = robot_map
        map_info = robot_map
        self.build_core_map(robot_map)

    def publish_to_core(self):
        while not rospy.is_shutdown():
            try:
                data_map = self.main_map
                data_map.data = self.core_map.reshape(self.main_map.info.height * self.main_map.info.width).tolist()
                data_map.header.stamp = rospy.Time.now()
                self.core_publisher.publish(data_map)
                self.rate.sleep()
            except Exception as e:
                print(e)

    def build_core_map(self, robot_map: nav_msgs.msg.OccupancyGrid):
        global map_info
        reshaped_map = np.asarray(robot_map.data).reshape(robot_map.info.height, robot_map.info.width)
        if self.core_map.shape != reshaped_map.shape:
            self.core_map = reshaped_map.copy()
        map1 = reshaped_map.copy()
        self.core_map[map1 == 100] = 100
        self.core_map[map1 == 0] = 0
        map_info = self.core_map


if __name__ == '__main__':
    core_map_builder = None
    map_info = nav_msgs.msg.OccupancyGrid()
    if len(sys.argv) > 5:
        _robot1 = sys.argv[1]
        _robot2 = sys.argv[2]
        _robot3 = sys.argv[3]
        _robot4 = sys.argv[4]
        core_map_builder = CoreMapBuilder(_robot1, _robot2, _robot3, _robot4)
    else:
        core_map_builder = CoreMapBuilder()
    rospy.spin()

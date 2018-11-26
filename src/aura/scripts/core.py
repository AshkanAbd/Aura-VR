#!/usr/bin/env python2.7

import rospy
import nav_msgs.msg
import numpy as np
import sys


class CoreMapBuilder:
    robot0 = 'robot0'
    robot1 = 'robot1'
    robot2 = 'robot2'
    robot3 = 'robot3'
    available_odom = {}
    available_robots = set()
    core_map = np.array([])
    robot_evolution = {}
    node_distance = {}
    close_list = set()
    main_map = None
    core_publisher = None
    rate = None

    def __init__(self, robot0='robot0', robot1='robot1', robot2='robot2', robot3='robot3'):
        self.robot0 = robot0
        self.robot1 = robot1
        self.robot2 = robot2
        self.robot3 = robot3
        rospy.init_node('core_builder')
        self.check_robots()
        for robot in self.available_robots:
            self.available_odom[robot] = rospy.wait_for_message('/' + robot + '/odom', nav_msgs.msg.Odometry)
            rospy.Subscriber('/' + robot + '/map', nav_msgs.msg.OccupancyGrid, self.get_robots_map, robot, 1000)
            rospy.Subscriber('/' + robot + '/odom', nav_msgs.msg.Odometry, self.get_odom, robot, 1000)
        self.core_publisher = rospy.Publisher('/core/map', nav_msgs.msg.OccupancyGrid, queue_size=100)
        self.rate = rospy.Rate(10)
        self.publish_to_core()

    def get_odom(self, odom, robot_id):
        self.available_odom[robot_id] = odom

    def get_robots_map(self, robot_map, robot_id):
        self.main_map = robot_map
        self.build_core_map(robot_map, self.available_odom[robot_id], robot_id)

    def publish_to_core(self):
        while not rospy.is_shutdown():
            if self.main_map is None or self.core_map.shape[0] == 0:
                continue
            data_map = self.main_map
            data_map.data = self.core_map.tolist()
            data_map.header.stamp = rospy.Time.now()
            self.core_publisher.publish(data_map)
            self.rate.sleep()

    def build_core_map(self, robot_map, odom, robot_id):
        map1 = np.asarray(robot_map.data)
        if self.core_map.shape != map1.shape:
            self.core_map = np.zeros(map1.shape, map1.dtype)
            self.core_map = self.core_map - 1
        new_zero_coo = np.where(map1 == 0)[0]
        new_one_coo = np.where(map1 == 100)[0]
        for coordinate in new_zero_coo:
            if coordinate in self.close_list:
                continue
            if self.core_map[coordinate] == -1:
                self.core_map[coordinate] = 0
                self.robot_evolution[coordinate] = robot_id
                self.node_distance[coordinate] = (robot_id, self.convert_from_robot_to_map(odom.pose.pose.position.y
                                                                                           , odom.pose.pose.position.x))
            elif self.core_map[coordinate] == 100:
                if self.robot_evolution[coordinate] == robot_id:
                    self.core_map[coordinate] = 0
                    self.node_distance[coordinate] = (robot_id,
                                                      self.convert_from_robot_to_map(odom.pose.pose.position.y
                                                                                     , odom.pose.pose.position.x))
                else:
                    distance = self.convert_from_robot_to_map(odom.pose.pose.position.y, odom.pose.pose.position.x)
                    if self.node_distance[coordinate][1] > distance:
                        self.close_list.add(coordinate)
                        self.core_map[coordinate] = 0
                        self.node_distance[coordinate] = (robot_id,
                                                          self.convert_from_robot_to_map(odom.pose.pose.position.y
                                                                                         , odom.pose.pose.position.x))
        for coordinate in new_one_coo:
            if coordinate in self.close_list:
                continue
            if self.core_map[coordinate] == -1:
                self.core_map[coordinate] = 100
                self.robot_evolution[coordinate] = robot_id
                self.node_distance[coordinate] = (robot_id, self.convert_from_robot_to_map(odom.pose.pose.position.y,
                                                                                           odom.pose.pose.position.x))
            elif self.core_map[coordinate] == 0:
                if self.robot_evolution[coordinate] == robot_id:
                    self.core_map[coordinate] = 100
                    self.node_distance[coordinate] = (robot_id,
                                                      self.convert_from_robot_to_map(odom.pose.pose.position.y,
                                                                                     odom.pose.pose.position.x))
                else:
                    distance = self.convert_from_robot_to_map(odom.pose.pose.position.y, odom.pose.pose.position.x)
                    if self.node_distance[coordinate][1] > distance:
                        self.close_list.add(coordinate)
                        self.node_distance[coordinate] = (robot_id,
                                                          self.convert_from_robot_to_map(odom.pose.pose.position.y,
                                                                                         odom.pose.pose.position.x))

    def convert_from_robot_to_map(self, robot_y, robot_x):
        map_x = (robot_x - self.main_map.info.origin.position.x) // self.main_map.info.resolution
        map_y = (robot_y - self.main_map.info.origin.position.y) // self.main_map.info.resolution
        return ((map_y - 1) * self.main_map.info.width) + map_x

    def check_robots(self):
        for i in [self.robot0, self.robot1, self.robot2, self.robot3]:
            robot = None
            try:
                robot = rospy.wait_for_message('/' + i + '/map', nav_msgs.msg.OccupancyGrid, 2)
            except Exception:
                print(i + ' is not available')
            if robot is not None:
                self.available_robots.add(i)


if __name__ == '__main__':
    core_map_builder = None
    if len(sys.argv) > 5:
        _robot1 = sys.argv[1]
        _robot2 = sys.argv[2]
        _robot3 = sys.argv[3]
        _robot4 = sys.argv[4]
        core_map_builder = CoreMapBuilder(_robot1, _robot2, _robot3, _robot4)
    else:
        core_map_builder = CoreMapBuilder()
    rospy.spin()

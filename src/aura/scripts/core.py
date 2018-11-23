#!/usr/bin/env python3

import rospy
import nav_msgs.msg
import numpy as np
import sys
import aura.msg


class CoreMapBuilder:
    robot0 = 'robot0'
    robot1 = 'robot1'
    robot2 = 'robot2'
    robot3 = 'robot3'
    core_map = np.array([])
    tolerance_zero = {}
    tolerance_one = {}
    robot_evolution = {}
    main_map = nav_msgs.msg.OccupancyGrid()
    core_publisher = None
    rate = None

    def __init__(self, robot0='robot0', robot1='robot1', robot2='robot2', robot3='robot3'):
        self.robot0 = robot0
        self.robot1 = robot1
        self.robot2 = robot2
        self.robot3 = robot3
        rospy.init_node('core_builder')
        rospy.Subscriber('/' + self.robot0 + '/map', nav_msgs.msg.OccupancyGrid, self.get_robots_map, '1')
        rospy.Subscriber('/' + self.robot1 + '/map', nav_msgs.msg.OccupancyGrid, self.get_robots_map, '2')
        rospy.Subscriber('/' + self.robot2 + '/map', nav_msgs.msg.OccupancyGrid, self.get_robots_map, '3')
        rospy.Subscriber('/' + self.robot3 + '/map', nav_msgs.msg.OccupancyGrid, self.get_robots_map, '4')
        rospy.Subscriber('/core/out_map', aura.msg.group_int, self.get_out_map)
        self.core_publisher = rospy.Publisher('/core/map', nav_msgs.msg.OccupancyGrid, queue_size=100)
        self.rate = rospy.Rate(5)
        self.publish_to_core()

    def get_out_map(self, out_map):
        for out in out_map.array:
            if self.core_map[out.data_int[0], out.data_int[1]] == -1:
                self.core_map[out.data_int[0], out.data_int[1]] = 100

    def get_robots_map(self, robot_map, robot_id):
        global map_info
        self.main_map = robot_map
        map_info = robot_map
        self.build_core_map(robot_map, robot_id)

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

    def build_core_map(self, robot_map, robot_id):
        global map_info
        reshaped_map = np.asarray(robot_map.data).reshape(robot_map.info.height, robot_map.info.width)
        if self.core_map.shape != reshaped_map.shape:
            self.core_map = reshaped_map.copy()
        map1 = reshaped_map.copy()
        new_zero = np.where(map1 == 0)
        new_zero_coo = []
        for i in range(len(new_zero[0])):
            new_zero_coo.append((new_zero[0][i], new_zero[1][i]))
        new_one = np.where(map1 == 100)
        new_one_coo = []
        for i in range(len(new_one[0])):
            new_one_coo.append((new_one[0][i], new_one[1][i]))
        for coordinate in new_zero_coo:
            if self.core_map[coordinate[0], coordinate[1]] == -1:
                self.core_map[coordinate[0], coordinate[1]] = 0
                self.robot_evolution[coordinate] = robot_id
            elif self.core_map[coordinate[0], coordinate[1]] == 100:
                if coordinate in self.robot_evolution and self.robot_evolution[coordinate] == robot_id:
                    self.core_map[coordinate[0], coordinate[1]] = 0
                else:
                    if coordinate not in self.tolerance_zero:
                        self.tolerance_zero[coordinate] = 1
                    else:
                        self.tolerance_zero[coordinate] += 1
        for coordinate in new_one_coo:
            if self.core_map[coordinate[0], coordinate[1]] == -1:
                self.core_map[coordinate[0], coordinate[1]] = 100
                self.robot_evolution[coordinate] = robot_id
            elif self.core_map[coordinate[0], coordinate[1]] == 0:
                if coordinate in self.robot_evolution and self.robot_evolution[coordinate] == robot_id:
                    self.core_map[coordinate[0], coordinate[1]] = 100
                else:
                    if coordinate not in self.tolerance_one:
                        self.tolerance_one[coordinate] = 1
                    else:
                        self.tolerance_one[coordinate] += 1
        for pair in self.tolerance_one.keys():
            if self.tolerance_one[pair] > 10 and self.core_map[pair[0], pair[1]] == 0:
                self.core_map[pair[0], pair[1]] = 100
        for pair in self.tolerance_zero.keys():
            if self.tolerance_zero[pair] > 10 and self.core_map[pair[0], pair[1]] == 100:
                self.core_map[pair[0], pair[1]] = 0
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

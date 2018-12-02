#!/usr/bin/env python2

import rospy
import std_msgs.msg
import nav_msgs.msg
import geometry_msgs.msg
import aura.msg
import math
import sys
import os
import numpy as np
import sklearn.cluster
import cv2 as cv

sys.path.insert(0, os.getcwd()[0:os.getcwd().index("/", 6) + 1] + 'Aura_VR/src/aura/libs')
import functions


class DetectFrontier:
    available_robots = set()
    core_map = nav_msgs.msg.OccupancyGrid()
    core_numpy = np.array([])
    robots_odom = {}
    frontier_publisher = None
    frontier_rate = None
    frontier_set = set()
    frontier_numpy = np.array([])

    def __init__(self, node_name):
        rospy.init_node(node_name)
        # self.check_robots('map', 'core')
        self.available_robots.add('aura1')
        for robot in self.available_robots:
            self.robots_odom[robot] = rospy.wait_for_message('/' + robot + '/odom', nav_msgs.msg.Odometry)
            rospy.Subscriber('/' + robot + '/map', nav_msgs.msg.OccupancyGrid, self.get_odoms, robot, 1000)
        self.get_core_map(rospy.wait_for_message('/core/map', nav_msgs.msg.OccupancyGrid))
        rospy.Subscriber('/core/map', nav_msgs.msg.OccupancyGrid, self.get_core_map, queue_size=1000)
        self.frontier_publisher = rospy.Publisher('/core/frontiers', aura.msg.group_int64, queue_size=1000)
        self.frontier_rate = rospy.Rate(2)

    def detect_frontiers(self):
        while not rospy.is_shutdown():
            for robot in self.robots_odom.keys():
                odom = self.robots_odom[robot]
                robot_pose = self.convert_from_robot_to_map(odom.pose.pose.position.y, odom.pose.pose.position.x)
                functions.detect_frontiers(robot_pose, self.frontier_set, self.core_numpy, self.core_map.info.width)
            frontier_numpy = np.empty((len(self.frontier_set), 2), np.int16)
            i = 0
            for it in self.frontier_set:
                frontier_numpy[i, 0] = int(it // self.core_map.info.width)
                frontier_numpy[i, 1] = int(it % self.core_map.info.width)
                i += 1
            dbscan = sklearn.cluster.DBSCAN(eps=3, min_samples=2).fit(frontier_numpy)
            density_map = {}
            for cluster in dbscan.labels_.tolist():
                if cluster in density_map.keys():
                    density_map[cluster] += 1
                else:
                    density_map[cluster] = 1
            self.publish_clusters(density_map, self.frontier_set, dbscan.labels_)
            self.frontier_rate.sleep()

    def publish_clusters(self, density_map, frontier_set, labels):
        keys = aura.msg.data_int64()
        values = aura.msg.data_int64()
        frontier = aura.msg.data_int64()
        label = aura.msg.data_int64()
        msg = aura.msg.group_int64()
        keys.data_int = density_map.keys()
        values.data_int = density_map.values()
        label.data_int = labels.tolist()
        frontier.data_int = np.array(list(frontier_set), dtype=np.int64).tolist()
        msg.array.append(keys)
        msg.array.append(values)
        msg.array.append(label)
        msg.array.append(frontier)
        self.frontier_publisher.publish(msg)

    def check_robots(self, map_topic, core_topic):
        robots = functions.get_topics(map_topic, core_topic)
        for i in robots:
            robot = None
            try:
                robot = rospy.wait_for_message('/' + i + '/map', nav_msgs.msg.OccupancyGrid, 2)
            except rospy.ROSException:
                print(i + ' is not available')
            if robot is not None:
                print(i + ' added to core')
                self.available_robots.add(i)

    # Callbacks
    def get_core_map(self, core):
        self.core_map = core
        self.core_numpy = np.array(core.data, dtype=np.float64)

    def get_odoms(self, odom, robot_id):
        self.robots_odom[robot_id] = odom

    # Converts
    def convert_from_robot_to_map(self, robot_y, robot_x):
        map_x = int(round((robot_x - self.core_map.info.origin.position.x) / self.core_map.info.resolution))
        map_y = int(round((robot_y - self.core_map.info.origin.position.y) / self.core_map.info.resolution))
        return (map_y * self.core_map.info.width) + map_x


if __name__ == '__main__':
    detect_frontier = DetectFrontier('frontier_detect')
    detect_frontier.detect_frontiers()
    rospy.spin()

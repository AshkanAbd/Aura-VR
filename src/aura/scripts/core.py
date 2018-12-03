#!/usr/bin/env python2.7

import rospy
import nav_msgs.msg
import numpy as np
import sys
import os
import threading
import tf.transformations
import geometry_msgs.msg
import math
import time

sys.path.insert(0, os.getcwd()[0:os.getcwd().index("/", 6) + 1] + 'Aura_VR/src/aura/libs')
import functions


class CoreMapBuilder:
    available_robots = set()
    available_odom = {}
    available_maps = {}
    robot_angle = {}
    core_map = np.array([])
    # robot_evolution = {}
    node_map = {}
    # node_distance = {}
    close_list = set()
    base_map_info = None
    base_map_header = None
    core_publisher = None
    map_shape = None
    rate = None
    cmd_controller = {}
    start = False
    publish_thread = None
    publish_map = None

    def __init__(self, map_topic, core_topic, node_name):
        rospy.init_node(node_name)
        self.check_robots(map_topic, core_topic)
        # for robot in self.available_robots:
        #     self.available_odom[robot] = rospy.wait_for_message('/' + robot + '/odom', nav_msgs.msg.Odometry)
        #     rospy.Subscriber('/' + robot + '/map', nav_msgs.msg.OccupancyGrid, self.get_robots_map, robot,
        #                      queue_size=100000)
        #     rospy.Subscriber('/' + robot + '/odom', nav_msgs.msg.Odometry, self.get_odom, robot, queue_size=100000)
        # self.core_publisher = rospy.Publisher('/core/map', nav_msgs.msg.OccupancyGrid, queue_size=1000)
        # self.rate = rospy.Rate(10)
        # self.publish_thread = threading.Thread(target=self.publish_to_core)
        # self.publish_thread.setName("core-publish")
        # self.publish_thread.start()
        self.rate = rospy.Rate(10)
        self.core_publisher = rospy.Publisher('/core/map', nav_msgs.msg.OccupancyGrid, queue_size=10000)
        for robot in self.available_robots:
            self.cmd_controller[robot] = (rospy.Publisher('/' + robot + '/cmd_vel'
                                                          , geometry_msgs.msg.Twist, queue_size=1000), rospy.Rate(10))
        self.start_building()

    def start_building(self):
        while not rospy.is_shutdown():
            for robot in self.available_robots:
                self.get_odom(rospy.wait_for_message('/' + robot + '/odom', nav_msgs.msg.Odometry), robot)
                self.available_maps[robot] = rospy.wait_for_message('/' + robot + '/map', nav_msgs.msg.OccupancyGrid)
            for robot in self.available_robots:
                self.build_core_map(self.available_maps[robot], self.available_odom[robot], robot)
            data_map = nav_msgs.msg.OccupancyGrid()
            data_map.header = self.base_map_header
            data_map.info = self.base_map_info
            data_map.data = self.publish_map.tolist()
            data_map.header.stamp = rospy.Time.now()
            self.core_publisher.publish(data_map)
            self.rate.sleep()

    def publish_to_core(self):
        while not rospy.is_shutdown():
            if not self.start or self.publish_map is None:
                continue
            data_map = nav_msgs.msg.OccupancyGrid()
            data_map.header = self.base_map_header
            data_map.info = self.base_map_info
            data_map.data = self.publish_map.tolist()
            data_map.header.stamp = rospy.Time.now()
            self.core_publisher.publish(data_map)
            self.rate.sleep()

    def build_core_map(self, robot_map, odom, robot_id):
        map1 = np.asarray(robot_map.data)
        if not self.start:
            self.core_map = np.zeros(map1.shape, map1.dtype)
            self.core_map = self.core_map - 1
            self.base_map_info = robot_map.info
            self.base_map_header = robot_map.header
            self.start = True
        if robot_id in self.robot_angle:
            if -5 > self.robot_angle[robot_id][1] > 5:
                self.build_cmd_thread(robot_id)
                return
        if self.base_map_info.height != robot_map.info.height and self.base_map_info.width != robot_map.info.width:
            print("Ignore")
            return
        robot_pose = self.convert_from_robot_to_map(odom.pose.pose.position.y, odom.pose.pose.position.x)
        new_zero_coo = np.where(map1 == 0)[0]
        new_one_coo = np.where(map1 == 100)[0]
        self.publish_map = self.core_map.copy()
        tmp = self.core_map.astype(np.float64).copy()
        functions.builder(tmp, new_zero_coo.tolist(), int(robot_pose), self.node_map, 0, robot_id,
                          self.base_map_info.width)
        # self.core_map = tmp.astype(np.int8).copy()
        functions.builder(tmp, new_one_coo.tolist(), int(robot_pose), self.node_map, 100, robot_id,
                          self.base_map_info.width)
        map_y, map_x = self.convert_from_robot_to_map1(odom.pose.pose.position.y, odom.pose.pose.position.x)
        for i in xrange(-1, 2):
            for j in xrange(-1, 2):
                coordinate = int(((map_y + i) * self.base_map_info.width) + (map_x + j))
                if tmp[coordinate] != -1:
                    tmp[coordinate] = 0
        self.core_map = tmp.astype(np.int8).copy()
        self.publish_map = self.core_map.copy()
        #############################################
        ############# replace with c++ ##############
        # for coordinate in new_zero_coo.tolist():
        #     # if coordinate in self.close_list:
        #     #     continue
        #     distance = abs(robot_pose - coordinate)
        #     if self.core_map[coordinate] == -1:
        #         self.core_map[coordinate] = 0
        #         self.node_map[coordinate] = (robot_id, distance)
        #         # self.robot_evolution[coordinate] = robot_id
        #         # self.node_distance[coordinate] = distance
        #     elif self.core_map[coordinate] == 100:
        #         # if self.robot_evolution[coordinate] == robot_id:
        #         if self.node_map[coordinate][0] == robot_id:
        #             # if self.node_distance[coordinate][1] > distance:
        #             self.core_map[coordinate] = 0
        #             # self.node_distance[coordinate] = distance
        #             self.node_map[coordinate] = (robot_id, distance)
        #         else:
        #             # if self.node_distance[coordinate] > distance:
        #             if self.node_map[coordinate][1] > distance:
        #                 # self.close_list.add(coordinate)
        #                 self.core_map[coordinate] = 0
        #                 # self.node_distance[coordinate] = distance
        #                 # self.robot_evolution[coordinate] = robot_id
        #                 self.node_map[coordinate] = (robot_id, distance)
        #     else:
        #         # if self.node_distance[coordinate] > distance:
        #         #     self.node_distance[coordinate] = distance
        #         if self.node_map[coordinate][1] > distance:
        #             self.node_map[coordinate] = (robot_id, distance)
        # for coordinate in new_zero_coo.tolist():
        #     # if coordinate in self.close_list:
        #     #     continue
        #     distance = abs(robot_pose - coordinate)
        #     if self.core_map[coordinate] == -1:
        #         self.core_map[coordinate] = 100
        #         self.node_map[coordinate] = (robot_id, distance)
        #         # self.robot_evolution[coordinate] = robot_id
        #         # self.node_distance[coordinate] = distance
        #     elif self.core_map[coordinate] == 0:
        #         # if self.robot_evolution[coordinate] == robot_id:
        #         if self.node_map[coordinate][0] == robot_id:
        #             # if self.node_distance[coordinate][1] > distance:
        #             self.core_map[coordinate] = 100
        #             # self.node_distance[coordinate] = distance
        #             del self.node_map[coordinate]
        #             self.node_map[coordinate] = (robot_id, distance)
        #         else:
        #             # if self.node_distance[coordinate] > distance:
        #             if self.node_map[coordinate][1] > distance:
        #                 # self.close_list.add(coordinate)
        #                 self.core_map[coordinate] = 100
        #                 # self.node_distance[coordinate] = distance
        #                 # self.robot_evolution[coordinate] = robot_id
        #                 self.node_map[coordinate] = (robot_id, distance)
        #     else:
        #         # if self.node_distance[coordinate] > distance:
        #         #     self.node_distance[coordinate] = distance
        #         if self.node_map[coordinate][1] > distance:
        #             new_id = self.node_map[coordinate][0]
        #             self.node_map[coordinate] = (new_one_coo, distance)
        ######################################################

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

    def build_cmd_thread(self, robot):
        cmd_thread = threading.Thread(target=self.move_forward, name=robot + "_move_forward", args=robot)
        cmd_thread.start()

    def move_forward(self, robot):
        print("moving forward")
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.3
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        for i in xrange(5):
            self.cmd_controller[robot][0].publish(twist)
            self.cmd_controller[robot][1].sleep()
        time.sleep(4.5)
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        for i in xrange(5):
            self.cmd_controller[robot][0].publish(twist)
            self.cmd_controller[robot][1].sleep()

    # Callbacks
    def get_odom(self, odom, robot_id):
        self.available_odom[robot_id] = odom
        q = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w
        )
        robot_yaw = tf.transformations.euler_from_quaternion(q)
        robot_angle = (robot_yaw[0] * 180 / math.pi, robot_yaw[1] * 180 / math.pi, robot_yaw[2] * 180 / math.pi)
        self.robot_angle[robot_id] = robot_angle

    def get_robots_map(self, robot_map, robot_id):
        self.build_core_map(robot_map, self.available_odom[robot_id], robot_id)

    #############
    # Converts
    def convert_from_robot_to_map(self, robot_y, robot_x):
        map_x = round((robot_x - self.base_map_info.origin.position.x) / self.base_map_info.resolution)
        map_y = round((robot_y - self.base_map_info.origin.position.y) / self.base_map_info.resolution)
        return (map_y * self.base_map_info.width) + map_x

    def convert_from_robot_to_map1(self, robot_y, robot_x):
        map_x = round((robot_x - self.base_map_info.origin.position.x) // self.base_map_info.resolution)
        map_y = round((robot_y - self.base_map_info.origin.position.y) // self.base_map_info.resolution)
        return map_y, map_x


if __name__ == '__main__':
    CoreMapBuilder('map', 'core', 'functions')
    rospy.spin()

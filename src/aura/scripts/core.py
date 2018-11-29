#!/usr/bin/env python2.7

import rospy
import nav_msgs.msg
import numpy as np
import sys
import threading
import tf.transformations
import math

sys.path.insert(0, '../libs/')
import core_builder


class CoreMapBuilder:
    available_odom = {}
    robot_angle = {}
    available_robots = set()
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
    start = False
    publish_thread = None

    def __init__(self, map_topic, core_topic, node_name):
        rospy.init_node(node_name)
        self.check_robots(map_topic, core_topic)
        for robot in self.available_robots:
            self.available_odom[robot] = rospy.wait_for_message('/' + robot + '/odom', nav_msgs.msg.Odometry)
            rospy.Subscriber('/' + robot + '/map', nav_msgs.msg.OccupancyGrid, self.get_robots_map, robot, 1000)
            rospy.Subscriber('/' + robot + '/odom', nav_msgs.msg.Odometry, self.get_odom, robot, 1000)
        self.core_publisher = rospy.Publisher('/core/map', nav_msgs.msg.OccupancyGrid, queue_size=100)
        self.rate = rospy.Rate(10)
        self.publish_thread = threading.Thread(target=self.publish_to_core)
        self.publish_thread.setName("core-publish")
        self.publish_thread.start()

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

    def publish_to_core(self):
        while not rospy.is_shutdown():
            if not self.start:
                continue
            data_map = nav_msgs.msg.OccupancyGrid()
            data_map.header = self.base_map_header
            data_map.info = self.base_map_info
            data_map.data = self.core_map.tolist()
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
        if -8 > self.robot_angle[robot_id][1] > 8:
            return
        if self.base_map_info.height != robot_map.info.height and self.base_map_info.width != robot_map.info.width:
            print("Ignore")
            return
        robot_pose = self.convert_from_robot_to_map(odom.pose.pose.position.y, odom.pose.pose.position.x)
        new_zero_coo = np.where(map1 == 0)[0]
        new_one_coo = np.where(map1 == 100)[0]
        tmp = self.core_map.astype(np.float64)
        core_builder.builder(tmp, new_zero_coo.tolist(), int(robot_pose), self.node_map, 0,
                             100, robot_id)
        core_builder.builder(tmp, new_one_coo.tolist(), int(robot_pose), self.node_map, 100,
                             0, robot_id)
        self.core_map = tmp.astype(np.int8)
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

    def convert_from_robot_to_map(self, robot_y, robot_x):
        map_x = round((robot_x - self.base_map_info.origin.position.x) / self.base_map_info.resolution)
        map_y = round((robot_y - self.base_map_info.origin.position.y) / self.base_map_info.resolution)
        return (map_y * self.base_map_info.width) + map_x

    def check_robots(self, map_topic, core_topic):
        robots = core_builder.get_topics(map_topic, core_topic)
        for i in robots:
            robot = None
            try:
                robot = rospy.wait_for_message('/' + i + '/map', nav_msgs.msg.OccupancyGrid, 2)
            except rospy.ROSException:
                print(i + ' is not available')
            if robot is not None:
                print(i + ' added to core')
                self.available_robots.add(i)


if __name__ == '__main__':
    CoreMapBuilder('map', 'core', 'core_builder')
    rospy.spin()

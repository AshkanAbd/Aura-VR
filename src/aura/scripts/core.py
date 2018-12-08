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
    auto_move_lock = False
    available_robots = set()
    available_odom = {}
    available_maps = {}
    robot_moving = {}
    robot_angle = {}
    core_map = np.array([])
    node_map = {}
    close_list = set()
    base_map_info = None
    base_map_header = None
    core_publisher = None
    map_shape = None
    rate = None
    cmd_controller = {}
    move_thread_lock = {}
    start = False
    auto_move_lock_thread = None
    node_name = None
    publish_map = None

    def __init__(self, map_topic, core_topic, node_name):
        self.node_name = node_name
        rospy.init_node(node_name)
        self.check_robots(map_topic, core_topic)
        for robot in self.available_robots:
            self.available_odom[robot] = rospy.wait_for_message('/' + robot + '/odom', nav_msgs.msg.Odometry)
            self.available_maps[robot] = rospy.wait_for_message('/' + robot + '/map', nav_msgs.msg.OccupancyGrid)
            rospy.Subscriber('/' + robot + '/map', nav_msgs.msg.OccupancyGrid, self.get_robots_map, robot,
                             queue_size=100000)
            rospy.Subscriber('/' + robot + '/odom', nav_msgs.msg.Odometry, self.get_odom, robot, queue_size=100000)
        # self.core_publisher = rospy.Publisher('/core/map', nav_msgs.msg.OccupancyGrid, queue_size=1000)
        # self.publish_thread = threading.Thread(target=self.publish_to_core)
        # self.publish_thread.setName("core-publish")
        # self.publish_thread.start()
        self.rate = rospy.Rate(10)
        self.core_publisher = rospy.Publisher('/core/map', nav_msgs.msg.OccupancyGrid, queue_size=10000)
        for robot in self.available_robots:
            self.cmd_controller[robot] = (
                rospy.Publisher('/' + robot + '/cmd_vel', geometry_msgs.msg.Twist, queue_size=1000), rospy.Rate(10))
        self.auto_move_lock_thread = threading.Thread(target=self.wait_for_click)
        self.auto_move_lock_thread.start()
        self.start_building()
        rospy.spin()

    def start_building(self):
        data_map = nav_msgs.msg.OccupancyGrid()
        while not rospy.is_shutdown():
            for robot in self.available_robots:
                # self.move_thread_lock[robot] = False
                self.build_core_map(self.available_maps[robot], self.available_odom[robot], robot)
                odom = self.available_odom[robot]
                map_y, map_x = self.convert_from_robot_to_map1(odom.pose.pose.position.y, odom.pose.pose.position.x)
                if self.core_map[int((map_y * self.base_map_info.width) + map_x)] != -1:
                    self.core_map[int((map_y * self.base_map_info.width) + map_x)] = 0
                if self.auto_move_lock:
                    self.check_robot_moving(robot, map_x, map_y, self.available_odom[robot].header.stamp.secs)
                data_map.header = self.base_map_header
                data_map.info = self.base_map_info
                data_map.header.stamp = rospy.Time.now()
                data_map.data = self.core_map.tolist()
                self.core_publisher.publish(data_map)
            # self.rate.sleep()

    def build_core_map(self, robot_map, odom, robot_id):
        map1 = np.asarray(robot_map.data)
        if not self.start:
            self.core_map = np.zeros(map1.shape, map1.dtype)
            self.core_map = self.core_map - 1
            self.base_map_info = robot_map.info
            self.base_map_header = robot_map.header
            self.start = True
        if robot_id in self.robot_angle:
            if self.robot_angle[robot_id][1] > 25:
                if not self.move_thread_lock[robot_id]:
                    self.build_cmd_thread(robot_id, 1)
                return
            if self.robot_angle[robot_id][1] < -25:
                if not self.move_thread_lock[robot_id]:
                    self.build_cmd_thread(robot_id, 2)
                return
        if self.base_map_info.height != robot_map.info.height and self.base_map_info.width != robot_map.info.width:
            print(robot_id + " ignored")
            return
        robot_pose = self.convert_from_robot_to_map(odom.pose.pose.position.y, odom.pose.pose.position.x)
        new_zero_coo = np.where(map1 == 0)[0]
        new_one_coo = np.where(map1 == 100)[0]
        # self.publish_map = self.core_map.copy()
        tmp = self.core_map.astype(np.float64).copy()
        functions.builder(tmp, new_zero_coo.tolist(), int(robot_pose), self.node_map, 0, robot_id,
                          self.base_map_info.width)
        # self.core_map = tmp.astype(np.int8).copy()
        functions.builder(tmp, new_one_coo.tolist(), int(robot_pose), self.node_map, 100, robot_id,
                          self.base_map_info.width)
        map_y, map_x = self.convert_from_robot_to_map1(odom.pose.pose.position.y, odom.pose.pose.position.x)
        sub_map = np.empty((3, 3), np.int8)
        for i in xrange(-1, 2):
            for j in xrange(-1, 2):
                sub_map[i + 1, j + 1] = tmp[int(((map_y + i) * self.base_map_info.width) + (map_x + j))]
        robot_ones = len(np.where(sub_map == 100)[0])
        if robot_ones == 1:
            for i in xrange(-1, 2):
                for j in xrange(-1, 2):
                    if tmp[int(((map_y + i) * self.base_map_info.width) + (map_x + j))] != -1:
                        tmp[int(((map_y + i) * self.base_map_info.width) + (map_x + j))] = 0
        elif robot_ones > 1:
            if not self.move_thread_lock[robot_id]:
                self.build_cmd_thread(robot_id, 3)
        self.core_map = tmp.astype(np.int8).copy()
        # self.publish_map = self.core_map.copy()

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
        # self.build_core_map(robot_map, self.available_odom[robot_id], robot_id)
        self.available_maps[robot_id] = robot_map

    def check_robot_moving(self, robot, map_x, map_y, time_stomp):
        if robot not in self.robot_moving:
            self.robot_moving[robot] = (time_stomp, map_x, map_y)
            return
        past = self.robot_moving[robot]
        past_time = past[0]
        distance = euclidean_distance(map_x, map_y, past[1], past[2])
        if distance < 2:
            if (time_stomp - past_time) < 15:
                pass
            else:
                if not self.move_thread_lock[robot]:
                    self.build_cmd_thread(robot, 3)
        else:
            self.robot_moving[robot] = (time_stomp, map_x, map_y)

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

    def wait_for_click(self):
        rospy.wait_for_message('/clicked_point', geometry_msgs.msg.PointStamped)
        self.auto_move_lock = True

    def check_robots(self, map_topic, core_topic):
        robots = functions.get_topics(map_topic, core_topic)
        for i in robots:
            robot = None
            try:
                robot = rospy.wait_for_message('/' + i + '/map', nav_msgs.msg.OccupancyGrid, 2)
            except rospy.ROSException:
                print(self.node_name + ": " + i + ' is not available')
            if robot is not None:
                print(self.node_name + ": " + i + ' added to core')
                self.move_thread_lock[i] = False
                self.available_robots.add(i)

    def build_cmd_thread(self, robot, flag):
        self.move_thread_lock[robot] = True
        cmd_thread = threading.Thread(target=move_forward, name=robot + "_move_forward",
                                      args=(robot, self.cmd_controller, self.move_thread_lock, flag))
        cmd_thread.start()

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


def move_with_check(robot, cmd_controller, move_thread_lock):
    odom = rospy.wait_for_message('/' + robot + '/odom', nav_msgs.msg.Odometry)
    core_map = rospy.wait_for_message('/core/map', nav_msgs.msg.OccupancyGrid)
    twist = geometry_msgs.msg.Twist()
    print(robot + " moving forward")
    twist.linear.x = 0.8
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    for i in xrange(5):
        cmd_controller[robot][0].publish(twist)
        cmd_controller[robot][1].sleep()
    time.sleep(5)
    twist = geometry_msgs.msg.Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    for i in xrange(5):
        cmd_controller[robot][0].publish(twist)
        cmd_controller[robot][1].sleep()
    new_odom = rospy.wait_for_message('/' + robot + '/odom', nav_msgs.msg.Odometry)
    old_y, old_x = convert_from_robot_to_map(core_map, odom.pose.pose.position.y, odom.pose.pose.position.x)
    new_y, new_x = convert_from_robot_to_map(core_map, new_odom.pose.pose.position.y, new_odom.pose.pose.position.x)
    if abs(old_y - new_y) < 2 and abs(old_x - new_x) < 2:
        twist = geometry_msgs.msg.Twist()
        print(robot + " moving backward")
        twist.linear.x = -0.8
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        for i in xrange(5):
            cmd_controller[robot][0].publish(twist)
            cmd_controller[robot][1].sleep()
        time.sleep(5)
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        for i in xrange(5):
            cmd_controller[robot][0].publish(twist)
            cmd_controller[robot][1].sleep()
    move_thread_lock[robot] = False


def move_forward(robot, cmd_controller, move_thread_lock, flag):
    if flag == 3:
        move_with_check(robot, cmd_controller, move_thread_lock)
        return
    twist = geometry_msgs.msg.Twist()
    if flag == 1:
        print(robot + " moving forward")
        twist.linear.x = 0.8
    elif flag == 2:
        print(robot + " moving backward")
        twist.linear.x = -0.8
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    for i in xrange(5):
        cmd_controller[robot][0].publish(twist)
        cmd_controller[robot][1].sleep()
    time.sleep(5)
    twist = geometry_msgs.msg.Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    for i in xrange(5):
        cmd_controller[robot][0].publish(twist)
        cmd_controller[robot][1].sleep()
    move_thread_lock[robot] = False


def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))


def convert_from_robot_to_map(core_map, robot_y, robot_x):
    map_x = round((robot_x - core_map.info.origin.position.x) / core_map.info.resolution)
    map_y = round((robot_y - core_map.info.origin.position.y) / core_map.info.resolution)
    return map_y, map_x


if __name__ == '__main__':
    CoreMapBuilder('map', 'core', 'core')
    rospy.spin()

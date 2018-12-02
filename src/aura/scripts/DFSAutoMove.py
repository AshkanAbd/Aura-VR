#!/usr/bin/env python3

import nav_msgs.msg
import numpy as np
import random
import std_msgs.msg
import block
import auto_move_base
import rospy
import aura.msg
import time
import geometry_msgs.msg
import math


class DFSAutoMove(auto_move_base.AutoMoveBase, object):
    block_array = []
    robot_block = None
    goal_x = -10000
    goal_y = -10000
    rotate_rate = None
    random_generator = None
    aborted_list = set()
    reshaped_map = None

    def __init__(self, namespace='robot0', node_name='AutoMoveBase', anonymous=True):
        super(DFSAutoMove, self).__init__(namespace, node_name, anonymous)
        # self.get_blocks(rospy.wait_for_message('/core/blocks', aura.msg.group_int))
        # rospy.Subscriber('/core/blocks', aura.msg.group_int, self.get_blocks)
        self.rotate_rate = rospy.Rate(10)
        self.random_generator = random.Random()
        self.get_map(rospy.wait_for_message('/core/map', nav_msgs.msg.OccupancyGrid))

    def get_blocks(self, blocks_array):
        temp_array = []
        for i in range(0, 256):
            block_obj = block.Block(i, blocks_array.array[i].data_int, self.map_info.info.height,
                                    self.map_info.info.width)
            temp_array.append(block_obj)
        self.block_array = temp_array
        self.robot_block = self.find_robot_block()

    def find_robot_block(self):
        robot_pose_y = self.robot_odometry.pose.pose.position.y
        robot_pose_x = self.robot_odometry.pose.pose.position.x
        robot_pose = self.convert_from_robot_to_map(robot_pose_y, robot_pose_x)
        y = robot_pose[0] // (self.map_info.info.height // 16)
        x = robot_pose[1] // (self.map_info.info.width // 16)
        robot_block_index = int((y * 16) + x)
        return robot_block_index

    def get_map(self, map):
        super(DFSAutoMove, self).get_map(map)
        self.reshaped_map = np.asarray(map.data, np.int8).reshape(map.info.height, map.info.width)
        if self.goal_x == -10000:
            return
        map_goal_y, map_goal_x = self.convert_from_robot_to_map(self.goal_y, self.goal_x)
        if self.reshaped_map[int(map_goal_y), int(map_goal_x)] != -1:
            self.client.cancel_all_goals()
        if not self.da_sihdir_da((self.goal_y, self.goal_x)):
            self.aborted_list.add((self.goal_x, self.goal_y))
            self.client.cancel_all_goals()

    def generating_goal(self, target):
        map_goal_x = target[1]
        map_goal_y = target[0]
        goal_y, goal_x = self.convert_from_map_to_robot(map_goal_y, map_goal_x)
        temp = aura.msg.data_int()
        temp.data_int = [goal_x, goal_y]
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.send_goal(goal_x, goal_y)
        print("GOAL PUBLISHED " + str(goal_x) + " , " + str(goal_y))
        return True

    def in_range(self, goal_x, goal_y):
        map_block = set()
        for i in xrange(-2, 3):
            for j in xrange(-2, 3):
                map_block.add((goal_x + i, goal_y + j))
        for pair in self.aborted_list:
            if pair in map_block:
                return False
        return True

    def get_neighbors(self, x, y):
        neighbors = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1), (x - 1, y + 1), (x + 1, y - 1), (x + 1, y + 1),
                     (x - 1, y - 1)]
        return neighbors

    def bfsihdir(self, blockindex):
        robot_y, robot_x = self.convert_from_robot_to_map(self.robot_odometry.pose.pose.position.y
                                                          , self.robot_odometry.pose.pose.position.x)
        visited = set()
        q = []
        p = []
        current = (robot_y, robot_x)
        if self.check_around(robot_x, robot_y):
            self.rotate()
        q.append(current)
        q.append((current[0] - 1, current[1]))
        q.append((current[0] + 1, current[1]))
        q.append((current[0], current[1] - 1))
        q.append((current[0], current[1] + 1))
        while len(q) != 0:
            current_sihdir = q.pop(0)
            visited.add(current_sihdir)
            for i in self.get_neighbors(int(current_sihdir[0]), int(current_sihdir[1])):
                if i not in q and i not in p:
                    if i not in visited:
                        if self.reshaped_map[int(i[0]), int(i[1])] == 100:
                            for k in xrange(-3, 3):
                                for l in range(-3, 3):
                                    visited.add((i[0] + k, i[1] + l))
                        elif self.reshaped_map[int(i[0]), int(i[1])] == -1:
                            if abs(i[0] - current[0]) + abs(i[1] - current[1]) >= 25:
                                if self.da_sihdir_da(i):
                                    if self.fucking_block(i):
                                        if self.in_range(i[1], i[0]):
                                            self.generating_goal(i)
                                        return
                                    else:
                                        p.append(i)
                            else:
                                q.append(i)
                        elif (self.reshaped_map[int(i[0]), int(i[1])] == 0):
                            q.append(i)

        min = 1000000
        index = -1
        i = 0
        for node in p:
            distance = math.sqrt(math.pow(node[1] - robot_x, 2) + math.pow(node[0] - robot_y, 2))
            if distance < min:
                min = distance
                index = i
            i += 1
        # if index != -1:
        self.generating_goal(p[index])

    def da_sihdir_da(self, k):
        a = []
        for i in xrange(-2, 2):
            for j in xrange(-2, 2):
                a.append(self.reshaped_map[int(k[0] + i), int(k[1] + j)])
        b = np.asarray(a)
        n_shown = np.where(b == 100)
        if len(n_shown[0]) == 0:
            return True
        return False

    def fucking_block(self, k):
        a = []
        for i in xrange(-4, 4):
            for j in xrange(-4, 4):
                a.append(self.reshaped_map[(k[0] + i), (k[1] + j)])
        b = np.asarray(a)
        n_shown = np.where(b == -1)
        if len(n_shown[0]) >= 15:
            return True
        return False

    # goal status--- PENDING=0--- ACTIVE=1-- PREEMPTED=2-- SUCCEEDED=3-- ABORTED=4-- REJECTED=5-- PREEMPTING=6-- RECALLING=7-- RECALLED=8-- LOST=9
    def goal_status(self, data1, data2):
        print(data1)
        if data1 == 4:
            map_goal_y, map_goal_x = self.convert_from_robot_to_map(self.goal_y, self.goal_x)
            temp = (self.goal_x, self.goal_y)
            if self.in_range(self.goal_x, self.goal_y):
                self.bfsihdir(self.robot_block)
            else:
                if self.check_around(map_goal_x, map_goal_y):
                    self.rotate()
                    self.send_goal(self.goal_x, self.goal_y)
                else:
                    self.aborted_list.add(temp)
                    self.bfsihdir(self.robot_block)
        else:
            self.bfsihdir(self.robot_block)

    def check_around(self, robot_x, robot_y):
        robot_around_matrix = self.reshaped_map[int(robot_y) - 2:int(robot_y) + 2, int(robot_x) - 2: int(robot_x) + 2]
        if robot_around_matrix.max() == 100:
            return True
        return False

    def rotate(self):
        print("start rotate")
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 2
        for i in xrange(5):
            self.cmd_publisher.publish(twist)
            self.rotate_rate.sleep()
        time.sleep(9.3)  # ~2.32 for 90 degree
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        for i in xrange(5):
            self.cmd_publisher.publish(twist)
            self.rotate_rate.sleep()

    # def start(self, block_index):
    #     if self.bfsihdir(block_index):
    #         return
    #     neighbors = [self.block_array[block_index].go_up(), self.block_array[block_index].go_down(),
    #                  self.block_array[block_index].go_left(), self.block_array[block_index].go_right()]
    #     for i in neighbors:
    #         if self.block_array[i].has_unkown():
    #             self.bfsihdir(i)
    #             print(neighbors)
    #             print(block_index)
    #             return
    #         else:
    #             return self.start(neighbors[0])

    def current_goal(self):
        return self.goal_x, self.goal_y

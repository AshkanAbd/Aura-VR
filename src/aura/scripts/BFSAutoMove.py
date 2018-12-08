#!/usr/bin/env python2

import nav_msgs.msg
import numpy as np
import random
import auto_move_base
import rospy
import aura.msg
import time
import geometry_msgs.msg
import math


def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))


class BFSAutoMove(auto_move_base.AutoMoveBase, object):
    goal_x = -10000
    goal_y = -10000
    rotate_rate = None
    random_generator = None
    aborted_list = set()
    robots_goal = {}
    victim_lock = False
    reshaped_map = None

    def __init__(self, namespace='robot0', node_name='AutoMoveBase', anonymous=True):
        super(BFSAutoMove, self).__init__(namespace, node_name, anonymous)
        rospy.Subscriber('/' + namespace + '/goto_victim', aura.msg.data_float, self.goto_victim)
        self.rotate_rate = rospy.Rate(10)
        self.random_generator = random.Random()
        self.get_map(rospy.wait_for_message('/core/map', nav_msgs.msg.OccupancyGrid))

    def get_map(self, map):
        super(BFSAutoMove, self).get_map(map)
        self.reshaped_map = np.asarray(map.data, np.int8).reshape(map.info.height, map.info.width)
        if self.goal_x == -10000:
            return
        if not self.victim_lock:
            map_goal_y, map_goal_x = self.convert_from_robot_to_map(self.goal_y, self.goal_x)
            if self.reshaped_map[int(map_goal_y), int(map_goal_x)] != -1:
                if self.verify_goal(map_goal_x, map_goal_y):
                    self.client.cancel_all_goals()
            if not self.da_sihdir_da((map_goal_y, map_goal_x)):
                self.aborted_list.add((self.goal_x, self.goal_y))
                self.client.cancel_all_goals()

    def verify_goal(self, x, y):
        r_y, r_x = self.convert_from_robot_to_map(self.robot_odometry.pose.pose.position.y,
                                                  self.robot_odometry.pose.pose.position.x)
        dis = math.sqrt(math.pow(r_y - y, 2) + math.pow(r_x - x, 2))
        if dis < 25:
            return True
        return False

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

    def bfsihdir(self):
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
                        # if self.reshaped_map[int(i[0]), int(i[1])] == 100:
                        #     for k in xrange(-3, 4):
                        #         for l in xrange(-3, 4):
                        #             visited.add((i[0] + k, i[1] + l))
                        # el
                        if self.reshaped_map[int(i[0]), int(i[1])] == -1:
                            if math.sqrt(((i[0] - current[0]) ** 2) + ((i[1] - current[1]) ** 2)) >= 6:
                                if self.da_sihdir_da(i):
                                    if self.fucking_block(i):
                                        if self.in_range(i[1], i[0]):
                                            if self.check_other_goals(i[1], i[0]):
                                                self.generating_goal(i)
                                                return
                                    else:
                                        p.append(i)
                            else:
                                q.append(i)
                        elif self.reshaped_map[int(i[0]), int(i[1])] == 0:
                            if self.zero_width(i):
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
        self.generating_goal(p[index])

    def zero_width(self, k):
        if self.reshaped_map[int(k[0]), int(k[1] + 1)] == 0 and self.reshaped_map[int(k[0]), int(k[1] - 1)] == 0:
            return True
        if self.reshaped_map[int(k[0] + 1), int(k[1])] == 0 and self.reshaped_map[int(k[0] - 1), int(k[1])] == 0:
            return True
        if self.reshaped_map[int(k[0] + 1), int(k[1] + 1)] == 0 and \
                self.reshaped_map[int(k[0] - 1), int(k[1] - 1)] == 0:
            return True
        if self.reshaped_map[int(k[0] - 1), int(k[1] + 1)] == 0 and \
                self.reshaped_map[int(k[0] + 1), int(k[1] - 1)] == 0:
            return True
        return False

    def da_sihdir_da(self, k):
        a = self.reshaped_map[int(k[0] - 2):int(k[0] + 3), int(k[1] - 2):int(k[1] + 3)]
        # for i in xrange(-2, 3):
        #     for j in xrange(-2, 3):
        #         a[i + 2, j + 2] = self.reshaped_map[int(k[0] + i), int(k[1] + j)]
        if a.max() != 100:
            return True
        return False

    def fucking_block(self, k):
        b = self.reshaped_map[int(k[0] - 3):int(k[0] + 4), int(k[1] - 3):int(k[0] + 4)]
        # for i in xrange(-3, 4):
        #     for j in xrange(-3, 4):
        #         b[i + 3, j + 3] = self.reshaped_map[(k[0] + i), (k[1] + j)]
        n_shown = np.where(b == -1)
        if len(n_shown[0]) >= 10:
            return True
        return False

    def check_other_goals(self, goal_x, goal_y):
        for robot in self.robots_goal.keys():
            if robot == self.namespace:
                continue
            y, x = self.convert_from_robot_to_map(self.robots_goal[robot][1], self.robots_goal[robot][0])
            if euclidean_distance(int(goal_x), int(goal_y), int(x), int(y)) < 30:
                return False
        return True

    # goal status--- PENDING=0--- ACTIVE=1-- PREEMPTED=2-- SUCCEEDED=3-- ABORTED=4-- REJECTED=5-- PREEMPTING=6--
    # RECALLING=7-- RECALLED=8-- LOST=9
    def goal_status(self, data1, data2):
        print(data1)
        if not self.victim_lock:
            if data1 == 4:
                map_goal_y, map_goal_x = self.convert_from_robot_to_map(self.goal_y, self.goal_x)
                temp = (self.goal_x, self.goal_y)
                if self.in_range(self.goal_x, self.goal_y):
                    self.bfsihdir()
                else:
                    if self.check_around(map_goal_x, map_goal_y):
                        self.rotate()
                        self.send_goal(self.goal_x, self.goal_y)
                    else:
                        self.aborted_list.add(temp)
                        self.bfsihdir()
            else:
                self.bfsihdir()

    def goto_victim(self, data):
        if data.data_float[0] == 1:
            self.victim_lock = True
            if abs(self.goal_y - data.data_float[2]) < 0.01 and abs(self.goal_x - data.data_float[1]) < 0.01:
                pass
            else:
                self.goal_x = data.data_float[1]
                self.goal_y = data.data_float[2]
                self.send_goal(data.data_float[1], data.data_float[2])
        else:
            self.victim_lock = False
            self.bfsihdir()

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

    def start(self):
        print("Wait for click point")
        rospy.wait_for_message('/clicked_point', geometry_msgs.msg.PointStamped)
        print("start!!!")
        self.bfsihdir()

    def all_goals(self, goals):
        for i in xrange(len(goals.sources)):
            self.robots_goal[goals.sources[i]] = goals.poses.array[i].data_float

    def current_goal(self):
        return self.goal_x, self.goal_y

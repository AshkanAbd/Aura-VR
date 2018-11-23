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


class DFSAutoMove(auto_move_base.AutoMoveBase, object):
    block_array = []
    robot_block = None
    goal_x = -10000
    goal_y = -10000
    rotate_rate = None
    random_generator = None
    aborted_list = set()

    def __init__(self, namespace='robot0', node_name='AutoMoveBase', anonymous=True):
        super(DFSAutoMove, self).__init__(namespace, node_name, anonymous)
        self.get_blocks(rospy.wait_for_message('/core/blocks', aura.msg.group_int))
        rospy.Subscriber('/core/blocks', aura.msg.group_int, self.get_blocks)
        self.rotate_rate = rospy.Rate(10)
        self.random_generator = random.Random()

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
        if self.goal_x == -10000:
            return
        map_goal_y, map_goal_x = self.convert_from_robot_to_map(self.goal_y, self.goal_x)
        reshape_map = np.asarray(map.data).reshape(map.info.height, map.info.width)
        if reshape_map[int(map_goal_y), int(map_goal_x)] == 100:
            self.client.cancel_all_goals()

    def generating_goal(self, block_index):
        n_shown = np.where(self.block_array[block_index].get_reshaped_block() == -1)
        if len(n_shown[0]) == 0:
            return False
        print(len(n_shown[0]))
        self.random_generator.seed(rospy.get_time() // 0.01)
        rand = self.random_generator.randint(0, len(n_shown[0]) - 1)
        map_goal_x = (self.block_array[block_index].block_width * self.block_array[block_index].column) + \
                     n_shown[1][rand]
        map_goal_y = (self.block_array[block_index].block_height * (self.block_array[block_index].row)) + \
                     n_shown[0][rand]
        goal_y, goal_x = self.convert_from_map_to_robot(map_goal_y, map_goal_x)
        temp = aura.msg.data_int()
        temp.data_int = [goal_x, goal_y]
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.send_goal(goal_x, goal_y)
        print("GOAL PUBLISHED " + str(goal_x) + " , " + str(goal_y))
        return True

    # goal status--- PENDING=0--- ACTIVE=1---PREEMPTED=2--SUCCEEDED=3--ABORTED=4---REJECTED=5--PREEMPTING=6---RECALLING=7---RECALLED=8---LOST=9
    def goal_status(self, data1, data2):
        print(data1)
        if data1 == 4:
            map_goal_y, map_goal_x = self.convert_from_robot_to_map(self.goal_y, self.goal_x)
            temp = (map_goal_x, map_goal_y)
            if temp in self.aborted_list:
                self.start(self.robot_block)
            else:
                if self.check_around():
                    self.rotate()
                    self.aborted_list.add(temp)
                    self.send_goal(self.goal_x, self.goal_y)
                else:
                    # send to core
                    self.start(self.robot_block)
        elif data1 == 2:
            pass
        else:
            self.start(self.robot_block)

    def check_around(self):
        robot_y, robot_x = self.convert_from_robot_to_map(self.robot_odometry.pose.pose.position.y
                                                          , self.robot_odometry.pose.pose.position.x)
        reshaped_map = np.asarray(self.map_info.data).reshape(self.map_info.info.height, self.map_info.info.width)
        robot_around_matrix = reshaped_map[int(robot_y) - 3:int(robot_y) + 3, int(robot_x) - 3: int(robot_x) + 3]
        if robot_around_matrix.argmin() == 100:
            print(robot_around_matrix)
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
        for i in range(5):
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
        for i in range(5):
            self.cmd_publisher.publish(twist)
            self.rotate_rate.sleep()

    def start(self, block_index):
        if self.generating_goal(block_index):
            return
        neighbors = [self.block_array[block_index].go_up(), self.block_array[block_index].go_down(),
                     self.block_array[block_index].go_left(), self.block_array[block_index].go_right()]
        # up = len(np.where(self.block_array[block_index].go_up()) == 100)
        # down = len(np.where(self.block_array[block_index].go_down()) == 100)
        # left = len(np.where(self.block_array[block_index].go_left()) == 100)
        # right = len(np.where(self.block_array[block_index].go_right()) == 100)
        # a = [up, down, left, right]
        # neighbors = np.sort(a[::-1])
        # o = np.where(neighbors) == up
        # j = np.where(neighbors) == down
        # k = np.where(neighbors) == right
        # l = np.where(neighbors) == left
        # np.put(neighbors, [o, j, l, k], [self.block_array[block_index].go_up(), self.block_array[block_index].go_down(),
        #                                  self.block_array[block_index].go_left(),
        #                                  self.block_array[block_index].go_right()])
        for i in neighbors:
            if self.block_array[i].has_unkown():
                self.generating_goal(i)
                print(neighbors)
                print(i)
                return
            else:
                return self.start(neighbors[0])

    def current_goal(self):
        return self.goal_x, self.goal_y

#!/usr/bin/env python3
import nav_msgs.msg
import numpy as np
import random
import block
import auto_move_base
import rospy
import aura.msg
import time
import geometry_msgs.msg


class DFSAutoMove(auto_move_base.AutoMoveBase):
    block_array = []
    robot_block = None
    goal_x = -10000
    goal_y = -10000
    rotate_rate = None

    random_generator = None

    def __init__(self, namespace='robot0', node_name='AutoMoveBase', anonymous=True):
        super().__init__(namespace, node_name, anonymous)
        self.get_blocks(rospy.wait_for_message('/core/blocks', aura.msg.group_int))
        rospy.Subscriber('/core/blocks', aura.msg.group_int, self.get_blocks)
        self.rotate_rate = rospy.Rate(10)
        self.random_generator = random.Random()

    def get_blocks(self, blocks_array: aura.msg.group_int):
        temp_array = []
        for i in range(0, 256):
            block_obj = block.Block(i, blocks_array.array[i].data_int, self.map_info.info.height,
                                    self.map_info.info.width)
            temp_array.append(block_obj)
        self.block_array = temp_array
        self.robot_block = self.find_robot_block()

    def find_robot_block(self) -> int:
        robot_pose_y = self.robot_odometry.pose.pose.position.y
        robot_pose_x = self.robot_odometry.pose.pose.position.x
        robot_pose = self.convert_from_robot_to_map(robot_pose_y, robot_pose_x)
        y = robot_pose[0] // (self.map_info.info.height // 16)
        x = robot_pose[1] // (self.map_info.info.width // 16)
        robot_block_index = int((y * 16) + x)
        return robot_block_index

    def get_map(self, map: nav_msgs.msg.OccupancyGrid):
        super().get_map(map)
        if self.goal_x == -10000:
            return
        map_goal_y, map_goal_x = self.convert_from_robot_to_map(self.goal_y, self.goal_x)
        reshape_map = np.asarray(map.data).reshape(map.info.height, map.info.width)
        # if reshape_map[int(map_goal_y), int(map_goal_x)] == 100:
        #     self.client.cancel_all_goals()
        #     self.generating_goal(self.robot_block)

    def generating_goal(self, block_index) -> bool:
        n_shown = np.where(self.block_array[block_index].get_reshaped_block() == -1)
        if (len(n_shown[0]) < 20): return False
        while True:
            self.random_generator.seed(rospy.get_time() // 0.01)
            rand = self.random_generator.randint(0, len(n_shown[0]) - 1)
            map_goal_x = (self.block_array[block_index].block_width * self.block_array[block_index].column) + \
                         n_shown[0][rand]
            map_goal_y = (self.block_array[block_index].block_height * (self.block_array[block_index].row)) + \
                         n_shown[1][
                             rand]
            goal_y, goal_x = self.convert_from_map_to_robot(map_goal_y, map_goal_x)
<<<<<<< HEAD
            temp = aura.msg.data_int()
            temp.data_int = [goal_x, goal_y]
=======
            temp = aura.msg.data_float()
            temp.data_float = [goal_x, goal_y]
>>>>>>> ee2b94427a9ac5d8aaa469d78503709c9b3349ab
            if temp not in self.black_list:
                break
        self.goal_x = goal_x
        self.goal_y = goal_y
        # self.client.cancel_goal()
        # self.client.wait_for_result()
        self.send_goal(goal_x, goal_y)
        print("GOAL PUBLISHED " + str(goal_x) + " , " + str(goal_y))
        return True

    # goal status--- PENDING=0--- ACTIVE=1---PREEMPTED=2--SUCCEEDED=3--ABORTED=4---REJECTED=5--PREEMPTING=6---RECALLING=7---RECALLED=8---LOST=9
    def goal_status(self, data1, data2):
        print(data1)
        if data1 == 4:
<<<<<<< HEAD
            temp = aura.msg.data_int()
            temp.data_int = [self.goal_x, self.goal_y]
=======
            temp = aura.msg.data_float()
            temp.data_float = [self.goal_x, self.goal_y]
>>>>>>> ee2b94427a9ac5d8aaa469d78503709c9b3349ab
            if temp not in self.black_list:
                self.rotate()
                self.send_goal(self.goal_x, self.goal_y)
                self.black_list_publisher.publish(temp)
                return
        self.start(self.robot_block)

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
        self.random_generator.shuffle(neighbors)
        for i in neighbors:
            if self.block_array[i].has_unkown():
                self.generating_goal(i)
                return
        self.start(neighbors[0])

    def current_goal(self) -> tuple:
        return self.goal_x, self.goal_y

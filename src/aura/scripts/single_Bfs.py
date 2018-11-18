import numpy as np
import random
import block
import auto_move_base
import rospy
import aura.msg
import time
import geometry_msgs.msg
import queue


class SingleBFS(auto_move_base.AutoMoveBase, object):
    list_of_nighbors = []
    visited = []
    goal_x = 10000
    goal_y = 10000
    q = []

    def __init__(self, namespace='robot0', node_name='AutoMoveBase'):
        super(SingleBFS, self).__init__(namespace, node_name)

    def get_nighbors(self):
        reshape_map = np.asarray(self.map_info.data).reshape(self.map_info.info.height, self.map_info.info.width)
        robot_y, robot_x = self.convert_from_robot_to_map(self.robot_odometry.pose.pose.position.y
                                                          , self.robot_odometry.pose.pose.position.x)

        self.list_of_nighbors = [reshape_map[robot_y - 1], reshape_map[robot_x],
                                 reshape_map[robot_y], reshape_map[robot_x + 1]]

    def generate_goal(self):
        self.list_of_nighbors
        goal_y, goal_x = self.convert_from_map_to_robot()
        temp = aura.msg.data_int()
        temp.data_int = [goal_x, goal_y]
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.send_goal(goal_x, goal_y)
        print("GOAL PUBLISHED " + str(goal_x) + " , " + str(goal_y))
        return True

    def start(self):
        reshape_map = np.asarray(self.map_info.data).reshape(self.map_info.info.height, self.map_info.info.width)
        robot_y, robot_x = self.convert_from_robot_to_map(self.robot_odometry.pose.pose.position.y
                                                          , self.robot_odometry.pose.pose.position.x)
        current = (robot_x,robot_y)
        self.visited.append(current)
        self.q.append(current)
        while len(self.q) != 0:
            if self.q.pop() == -1:
                # self.generate_goal()
                return
            for i in self.list_of_nighbors:
                if i not in self.visited and i not in self.q:
                    if i != 100:
                        self.q.append(i)

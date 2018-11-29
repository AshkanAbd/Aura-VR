import numpy as np
import random
import block
import auto_move_base
import rospy
import aura.msg
import time
import geometry_msgs.msg


class SingleBFS(auto_move_base.AutoMoveBase, object):
    goal_x = 10000
    goal_y = 10000

    def __init__(self, namespace='robot0', node_name='AutoMoveBase'):
        super(SingleBFS, self).__init__(namespace, node_name)


    def get_nighbors(self, y, x):
        neighbors = [(y - 10, x),
                     (y, x + 10),
                     (y + 10, x),
                     (y, x - 10)]
        return neighbors

    def generate_goal(self, directon):
        self.goal_y, self.goal_x = self.convert_from_map_to_robot(directon[0], directon[1])
        self.send_goal(self.goal_x, self.goal_y)
        print("GOAL PUBLISHED " + str(self.goal_x) + " , " + str(self.goal_y))
        return True

    # goal status--- PENDING=0--- ACTIVE=1---PREEMPTED=2--SUCCEEDED=3--ABORTED=4---REJECTED=5--PREEMPTING=6---RECALLING=7---RECALLED=8---LOST=9
    def goal_status(self, data1, data2):
        print(data1)
        self.start()

    def start(self):
        robot_y, robot_x = self.convert_from_robot_to_map(self.robot_odometry.pose.pose.position.y
                                                          , self.robot_odometry.pose.pose.position.x)
        current = (robot_y, robot_x)
        visited = []
        q = []
        visited.append(current)
        q.append(current)
        while len(q) != 0:
            current_node = q.pop(0)
            if self.reshape_map[int(current_node[0]), int(current_node[1])] == -1:
                self.generate_goal(current_node)
                return
            for i in self.get_nighbors(int(current_node[0]), int(current_node[1])):
                if (i not in visited) and (i not in q):
                    if self.reshape_map[int(i[0]), int(i[1])] != 100:
                        q.append(i)
            visited.append(current_node)

    def get_map(self, map):
        super().get_map(map)
        self.reshape_map = np.asarray(self.map_info.data).reshape(self.map_info.info.height, self.map_info.info.width)

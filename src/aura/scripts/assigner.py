#!/usr/bin/env python2

import rospy
import aura.msg
import numpy as np
import move_base_msgs.msg
import geometry_msgs.msg
import nav_msgs.msg
import actionlib
import sys
import os


sys.path.insert(0, os.getcwd()[0:os.getcwd().index("/", 6) + 1] + 'Aura_VR/src/aura/libs')
import functions


class Assigner:
    available_robots = set()
    core_map = None
    clusters = {}
    move_base_goals = {}
    action_clients = {}
    frontier_map = {}
    frontier_set = set()

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.check_robots('map', 'core')
        for namespace in self.available_robots:
            self.setup_move_base(namespace)
        self.core_map = rospy.wait_for_message('/core/map', nav_msgs.msg.OccupancyGrid)
        rospy.Subscriber('/core/frontiers', aura.msg.group_int64, self.get_frontiers, queue_size=1000)

    def send_goal(self, robot, x, y):
        goal = geometry_msgs.msg.PoseStamped()
        goal.header.frame_id = "/map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1
        self.move_base_goals[robot].target_pose = goal
        self.action_clients[robot].send_goal(self.move_base_goals[robot],self.goal_status)

    def goal_status(self,data1,data2):
        pass

    def build_frontier_map(self, keys, values):
        self.frontier_map = {}
        for i in xrange(len(keys)):
            self.frontier_map[keys[i]] = values[i]

    def build_clusters(self, keys, nodes):
        for i in xrange(len(keys)):
            self.clusters[keys[i]] = nodes[i]

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

    def setup_move_base(self, namespace):
        self.action_clients[namespace] = actionlib.SimpleActionClient('/' + namespace + '/move_base_node',
                                                                      move_base_msgs.msg.MoveBaseAction)
        self.action_clients[namespace].wait_for_server()
        self.move_base_goals[namespace] = move_base_msgs.msg.MoveBaseGoal()

    # Callback
    def get_frontiers(self, frontiers):
        self.build_frontier_map(frontiers.array[0], frontiers.array[1])
        self.build_clusters(frontiers.array[2], frontiers.array[3])

    # Converts
    def convert_from_robot_to_map(self, robot_y, robot_x):
        map_x = round((robot_x - self.core_map.info.origin.position.x) / self.core_map.info.resolution)
        map_y = round((robot_y - self.core_map.nfo.origin.position.y) / self.core_map.info.resolution)
        return (map_y * self.core_map.info.width) + map_x


if __name__ == '__main__':
    pass

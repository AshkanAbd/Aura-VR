#!/usr/bin/env python2.7

import rospy
import aura.msg
import nav_msgs.msg
import sys
import os

sys.path.insert(0, os.getcwd()[0:os.getcwd().index("/", 5) + 1] + 'Aura_VR/src/aura/libs')
import functions


class GoalHandler:
    available_robots = set()
    goal_publisher = None
    goals = {}
    rate = None
    node_name = None

    def __init__(self, node_name):
        self.node_name = node_name
        rospy.init_node(node_name)
        self.check_robots('map', 'core')
        self.goal_publisher = rospy.Publisher('/core/goal_publisher', aura.msg.multi_goal, queue_size=1000)
        self.rate = rospy.Rate(2)
        for robot in self.available_robots:
            rospy.Subscriber('/' + robot + '/goal_pose', aura.msg.goal, self.goal_pose)
        self.publish_goals()

    def check_robots(self, map_topic, core_topic):
        robots = functions.get_topics(map_topic, core_topic)
        for i in robots:
            robot = None
            try:
                robot = rospy.wait_for_message('/' + i + '/map', nav_msgs.msg.OccupancyGrid, 2)
            except rospy.ROSException:
                print(self.node_name + ": " + i + ' is not available')
            if robot is not None:
                print(self.node_name + ": " + i + ' found')
                self.available_robots.add(i)

    def publish_goals(self):
        while not rospy.is_shutdown():
            if len(self.goals.keys()) == 0:
                continue
            goals_pose = aura.msg.multi_goal()
            for robot in self.goals.keys():
                goals_pose.sources.append(robot)
                data = aura.msg.data_float()
                data.data_float = self.goals[robot]
                goals_pose.poses.array.append(data)
            self.goal_publisher.publish(goals_pose)
            self.rate.sleep()

    def goal_pose(self, pose):
        self.goals[pose.source] = [pose.pose.data_float[0], pose.pose.data_float[1]]


if __name__ == '__main__':
    GoalHandler('goal_handler')
    rospy.spin()

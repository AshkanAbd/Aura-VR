#!/usr/bin/env python3

import rospy
import actionlib
import nav_msgs.msg
import move_base_msgs.msg
import geometry_msgs.msg
import numpy as np
import aura.msg
import math


def get_robot_odom(odometry: nav_msgs.msg.Odometry):
    global robot_pose_x, robot_pose_y, robot_odometry, is_published
    robot_odometry = odometry
    robot_pose_x = odometry.pose.pose.position.x
    robot_pose_y = odometry.pose.pose.position.y
    is_published[0] = True


def get_map(map: nav_msgs.msg.OccupancyGrid):
    global map_width, map_height, map_info, is_published
    map_info = map
    map_width = map.info.width
    map_height = map.info.height
    is_published[1] = True


def convert_from_robot_to_map(robot_y, robot_x):
    global map_info
    map_x = (robot_x - map_info.info.origin.position.x) // map_info.info.resolution
    map_y = (robot_y - map_info.info.origin.position.y) // map_info.info.resolution
    return map_y, map_x


def convert_from_map_to_robot(map_y, map_x):
    global map_info
    robot_x = ((map_x) * map_info.info.resolution) + map_info.info.origin.position.x
    robot_y = ((map_y) * map_info.info.resolution) + map_info.info.origin.position.y
    return robot_y, robot_x


def cluster(group: aura.msg.group):
    global robot_pose_x, robot_pose_y, map_height, map_width, blocks
    blocks = group
    robot_block_index = find_cluster()


def find_cluster():
    global map_height, map_width, robot_pose_y, robot_pose_x
    while False in is_published: pass
    robot_pose = convert_from_robot_to_map(robot_pose_y, robot_pose_x)
    y = robot_pose[0] // (map_height // 16)
    x = robot_pose[1] // (map_width // 16)
    robot_block_index = int((y * 16) + x)
    return robot_block_index


def setup_move_base():
    global client, move_base_goal
    client = actionlib.SimpleActionClient('/' + namespace + '/move_base', move_base_msgs.msg.MoveBaseAction)
    client.wait_for_server()
    move_base_goal = move_base_msgs.msg.MoveBaseGoal()


# goal status--- PENDING=0--- ACTIVE=1---PREEMPTED=2--SUCCEEDED=3--ABORTED=4---REJECTED=5--PREEMPTING=6---RECALLING=7---RECALLED=8---LOST=9
def goal_status(data1, data2):
    pass


def move_base_clinet(goal_x, goal_y):
    global client, move_base_goal
    client.cancel_all_goals()
    goal = geometry_msgs.msg.PoseStamped()
    goal.header.frame_id = "/map"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = goal_x
    goal.pose.position.y = goal_y
    goal.pose.orientation.w = 1
    move_base_goal.target_pose = goal
    client.send_goal(move_base_goal, goal_status)


client = None
move_base_goal = None
is_published = [False, False]
robot_odometry = nav_msgs.msg.Odometry
map_info = nav_msgs.msg.OccupancyGrid.info
current_goal_x = 10000
current_goal_y = 10000
robot_pose_x = None
robot_pose_y = None
map_width = None
map_height = None
blocks = aura.msg.group

if __name__ == '__main__':
    namespace = 'robot0'
    rospy.init_node('amir_auto_move')
    setup_move_base()
    rospy.Subscriber('/core/map', nav_msgs.msg.OccupancyGrid, get_map)
    rospy.Subscriber('/core/blocks', aura.msg.group, cluster)
    rospy.Subscriber('/' + namespace + '/odom', nav_msgs.msg.Odometry, get_robot_odom)
    rospy.spin()

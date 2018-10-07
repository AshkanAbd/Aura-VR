#!/usr/bin/env python3

import rospy
import actionlib
import nav_msgs.msg
import move_base_msgs.msg
import geometry_msgs.msg
import numpy as np
import aura.msg

map = nav_msgs.msg.OccupancyGrid
robot_odometry = nav_msgs.msg.Odometry
map_info = nav_msgs.msg.OccupancyGrid.info
goal = geometry_msgs.msg.PoseStamped()
corent_goal_x = 10000
corent_goal_y = 10000
robot_x = None
robot_y = None
map_x = None
map_y = None
group = aura.msg.group
cluster_map = group.array


def get_robot_odom(odometry: nav_msgs.msg.Odometry):
    global robot_x, robot_y, robot_odometry, is_published
    robot_odometry = odometry
    robot_x = odometry.pose.pose.position.x
    robot_y = odometry.pose.pose.position.y
    is_published[0] = True
    # print((robot_y, robot_x))
    # try:
    #     print(convert_from_robot_to_map(robot_y, robot_x))
    #
    # except:
    #     robot_x = None
    #     robot_y = None


def get_map(map: nav_msgs.msg.OccupancyGrid):
    global map_x, map_y, map_info, is_published
    map_info = map
    map_x = map.info.width
    map_y = map.info.height
    is_published[1] = True
    # print((map_y, map_x))


# def find_cluster():
#     global map_y, map_x, robot_y, robot_x, is_published, cluster_map
#     # y = int(robot_y / (map_y / 16))
#     while False in is_published: pass
#     cluster_map = group.array
#     a = convert_from_robot_to_map(robot_y,robot_x)
#     x = a[0] // (map_y // 16)
#     y = a[1] // (map_x // 16)
#     s = ((y - 1) * 16) + (16 - x)


# def convert_from_robot_to_local_map(robot_y, robot_x):
#     global local_map
#     map_x = (robot_x - local_map.info.origin.position.x) // local_map.info.resolution
#     map_y = (robot_y - local_map.info.origin.position.y) // local_map.info.resolution
#     return map_y, map_x


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


# def get_up():
#     global current_robot_x ,current_robot_y
#     current_robot_x = None
#     current_robot_y = None
#     goal_x =

def cluster(group: aura.msg.group):
    global cluster_map, robot_x, robot_y,map_y,map_x,is_published
    cluster_map = group.array
    while False in is_published: pass
    a = convert_from_robot_to_map(robot_y, robot_x)
    x = a[0] // (map_y // 16)
    y = a[1] // (map_x // 16)
    s = int(((y - 1) * 16) + (16 - x))
    # print(s)
    print(cluster_map[s])
    # print(s)
    # robot_cluster = np.asarray(cluster_map[s]).reshape(62,62)
    # print(robot_cluster)
    # is_published[2] = True
    # cluster_map = np.asarray(cluster_map).reshape(16, 16)
    # print(cluster_map[0][1])
    # cluster_map = np.asarray(group.array).reshape()


# def generate_goal():


def setup_move_base():
    global client, move_base_goal
    client = actionlib.SimpleActionClient('/' + name + '/move_base', move_base_msgs.msg.MoveBaseAction)
    client.wait_for_server()
    move_base_goal = move_base_msgs.msg.MoveBaseGoal()


def move_base_clinet(goal_x, goal_y):
    global client, move_base_goal
    goal = geometry_msgs.msg.PoseStamped()
    goal.header.frame_id = "/map"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = goal_x
    goal.pose.position.y = goal_y
    goal.pose.orientation.w = 1
    move_base_goal.target_pose = goal
    client.send_goal(move_base_goal)


client = None
move_base_goal = None
is_published = [False , False ]

if __name__ == '__main__':
    name = 'robot0'
    rospy.init_node('amir_auto_move')
    setup_move_base()
    print('hey')
    rospy.Subscriber('/core/map', nav_msgs.msg.OccupancyGrid, get_map)
    rospy.Subscriber('/core/blocks', aura.msg.group, cluster)
    rospy.Subscriber('/' + name + '/odom', nav_msgs.msg.Odometry, get_robot_odom)
    # find_cluster()
    rospy.spin()

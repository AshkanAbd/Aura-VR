#!/usr/bin/env python3

import rospy
import actionlib
import nav_msgs.msg
import move_base_msgs.msg
import geometry_msgs.msg
import numpy as np
import aura.msg
import math
import random
import block
import core
import auto_move_base
import DFSAutoMove

client = None
move_base_goal = None
is_published = [False, False]
robot_odometry = nav_msgs.msg.Odometry
map_info = nav_msgs.msg.OccupancyGrid.info
current_goal_x = 10000
current_goal_y = 10000
robot_pose_x = None
robot_pose_y = None
robot_clu = None
map_width = None
map_height = None
robot_block_ind = None
list_of_block = None
blocks = aura.msg.group_int


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


def blocks_info(group: aura.msg.group_int):
    global list_of_block
    list_of_block = []
    blocks = group
    x = list(range(len(blocks.array)))
    while False in is_published: pass
    for i in x:
        block_obj = block.Block(i, blocks.array[i].data, map_height, map_width)
        list_of_block.append(block_obj)


if __name__ == '__main__':
    namespace = 'robot0'
    # rospy.init_node('amir_auto_move')
    # rospy.Subscriber('/core/map', nav_msgs.msg.OccupancyGrid, get_map)
    # rospy.Subscriber('/core/blocks', aura.msg.group, blocks_info)
    # rospy.Subscriber('/' + namespace + '/odom', nav_msgs.msg.Odometry, get_robot_odom)
    # rospy.spin()
    dfs_auto_move = DFSAutoMove.DFSAutoMove(namespace)
    dfs_auto_move.start(dfs_auto_move.robot_block)
    rospy.spin()

# def cluster(group: aura.msg.group):
#     global robot_pose_x, robot_pose_y, map_height, map_width, blocks, blocks_list
#     blocks = group
#     robot_block_index = find_robot_cluster()
#     upper_block_index = get_up_cluster()
#     lower_block_index = get_down_cluster()
#     left_block_index = get_left_cluster()
#     right_block_index = get_right_cluster()
#     blocks_list = [upper_block_index, lower_block_index, right_block_index, left_block_index]


# def get_up_cluster():
#     global robot_block_index, y, x, upper_cluster
#     upper_cluster = int((y - 1) * 16 + x)
#     return upper_cluster
#
#
# def get_down_cluster():
#     global robot_block_index, y, x, lower_cluster
#     lower_cluster = int((y + 1) * 16 + x)
#     return lower_cluster
#
#
# def get_right_cluster():
#     global robot_block_index, y, x, right_cluster
#     right_cluster = int(((y) * 16 + x) + 1)
#     return right_cluster
#
#
# def get_left_cluster():
#     global robot_block_index, y, x, left_cluster
#     left_cluster = int(((y) * 16 + x) - 1)
#     return left_cluster


# def generating_goal():
#     global robot_block, map_height, map_width, robot_clu, robot_block_ind, robot_clu, is_published, robot_pose_x, robot_pose_y
#     robot_pose = convert_from_robot_to_map(robot_pose_y, robot_pose_x)
#     y = robot_pose[0] // (map_height // 16)
#     x = robot_pose[1] // (map_width // 16)
#     y1 = (map_width // 16) * (x)
#     x1 = (map_height // 16) * (y)
#     # roboty, robotx = convert_from_robot_to_map(robot_odometry.pose.pose.position.y, robot_odometry.pose.pose.position.x)
#     n_shown = np.where(reshaped_robot_cluster == -1)
#     rand_celly = random.randint(0, len(n_shown[0]))
#     rand_cellx = random.randint(0, len(n_shown[1]))
#     if (len(n_shown[0] != 0)):
#         goal_x, goal_y = convert_from_map_to_robot(y1 + n_shown[0][rand_celly], x1 + n_shown[1][rand_cellx])
#         move_base_clinet(goal_y, goal_x)
#
#
# def sending_goal():
#     global robot_block_ind, is_published, map_height, map_width, reshaped_robot_cluster, blo
#     while False in is_published: pass
#     robot_block_ind = find_robot_cluster()
#     print(robot_block_ind)
#     reshaped_robot_cluster = np.asarray(blocks.array[robot_block_ind].data).reshape(map_width // 16, map_width // 16)
#     if len(reshaped_robot_cluster[reshaped_robot_cluster == -1]) > 0:
#         # print(robot_block_ind)
#         generating_goal()
#     else:
#         random.shuffle(blocks_list)
#         robot_block_ind = blocks_list[0]
#         print(blocks_list)
#         print(robot_block_ind)
#         reshaped_robot_cluster = np.asarray(blocks.array[robot_block_ind].data).reshape(map_width // 16,
#                                                                                        map_width // 16)
#         if len(reshaped_robot_cluster[reshaped_robot_cluster == -1]) > 0:
#             generating_goal()

# robot_pose = convert_from_robot_to_map(robot_pose_y, robot_pose_x)
# y = robot_pose[0] // (map_height // 16)
# x1 = (map_height // 16) * (y)
# reshaped_robot_cluster = np.asarray(blocks.array[robot_block_ind].data).reshape(map_width // 16, map_width // 16)
# roboty, robotx = convert_from_robot_to_map(robot_odometry.pose.pose.position.y, robot_odometry.pose.pose.position.x)

# x = robot_pose[1] // (map_width // 16)
# y1 = (map_width // 16) * (x)
# n_shown = np.where(reshaped_robot_cluster == -1)
# rand_celly = random.randint(0, len(n_shown[0]))
# rand_cellx = random.randint(0, len(n_shown[1]))
# if(len(n_shown[0]!=0)):
#     goal_x, goal_y = convert_from_map_to_robot(y1 + n_shown[0][rand_celly], x1 + n_shown[1][rand_cellx])
#     move_base_clinet(goal_y, goal_x)
# else:
#     print(blocks_list)
#     robot_block_ind = random.choice(blocks_list)


#
# # goal status--- PENDING=0--- ACTIVE=1---PREEMPTED=2--SUCCEEDED=3--ABORTED=4---REJECTED=5--PREEMPTING=6---RECALLING=7---RECALLED=8---LOST=9
# def goal_status(data1, data2):
#     print(data1)
#     if (data1 == 1):
#         pass
#     elif (data1 == 2 or data1 == 3 or data1 == 4 or data1 == 5):
#         pass
#     else:
#         pass


# def setup_move_base():
#     global client, move_base_goal
#     client = actionlib.SimpleActionClient('/' + namespace + '/move_base', move_base_msgs.msg.MoveBaseAction)
#     client.wait_for_server()
#     move_base_goal = move_base_msgs.msg.MoveBaseGoal()
#
#
#
# def move_base_clinet(goal_x, goal_y):
#     global client, move_base_goal
#     client.cancel_all_goals()
#     goal = geometry_msgs.msg.PoseStamped()
#     goal.header.frame_id = "/map"
#     goal.header.stamp = rospy.Time.now()
#     goal.pose.position.x = goal_x
#     goal.pose.position.y = goal_y
#     goal.pose.orientation.w = 1
#     move_base_goal.target_pose = goal
#     client.send_goal(move_base_goal, goal_status)

# def find_robot_cluster():
#     global map_height, map_width, robot_pose_y, robot_pose_x, y, x, is_published
#     while False in is_published: pass
#     robot_pose = convert_from_robot_to_map(robot_pose_y, robot_pose_x)
#     y = robot_pose[0] // (map_height // 16)
#     x = robot_pose[1] // (map_width // 16)
#     robot_block_index = int((y * 16) + x)
#     return robot_block_index


# def convert_from_robot_to_map(robot_y, robot_x):
#     global map_info
#     map_x = (robot_x - map_info.info.origin.position.x) // map_info.info.resolution
#     map_y = (robot_y - map_info.info.origin.position.y) // map_info.info.resolution
#     return map_y, map_x
#
#
# def convert_from_map_to_robot(map_y, map_x):
#     global map_info
#     robot_x = (map_x * map_info.info.resolution) + map_info.info.origin.position.x
#     robot_y = (map_y * map_info.info.resolution) + map_info.info.origin.position.y
#     return robot_y, robot_x

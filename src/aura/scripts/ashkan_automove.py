#!/usr/bin/env python3.5

from __future__ import print_function
import rospy
import actionlib
import actionlib.msg
import actionlib_msgs.msg
import actionlib_msgs
import move_base
import move_base_msgs
import move_base_msgs.msg
import numpy as np
import nav_msgs.msg
import tf.transformations
import tf.msg
import math
import std_msgs.msg
import geometry_msgs.msg
import geometry_msgs


def get_map(core_map):
    global main_map, reshaped_map, start
    main_map = core_map
    reshaped_map = np.asarray(core_map.data).reshape(core_map.info.height, core_map.info.width)


def get_odom(robot_odom):
    global main_odom, robot_angle, start
    main_odom = robot_odom
    q = (
        robot_odom.pose.pose.orientation.x,
        robot_odom.pose.pose.orientation.y,
        robot_odom.pose.pose.orientation.z,
        robot_odom.pose.pose.orientation.w
    )
    robot_angle = (tf.transformations.euler_from_quaternion(q)[2]) * 180 / math.pi


def goal_result(goal_status, data):
    # goal status--- PENDING=0--- ACTIVE=1---PREEMPTED=2--SUCCEEDED=3--ABORTED=4---REJECTED=5--PREEMPTING=6---RECALLING=7---RECALLED=8---LOST=9
    print('goal status = ', end='')
    print(goal_status)


def hot_victim(position):
    x = position.data[0]
    y = position.data[1]
    w = position.data[2]
    h = position.data[3]
    victim_lock = True


def build_victim_pose(x, y, w, h):
    robot_map_y, robot_map_x = convert_from_robot_to_map(main_odom.pose.pose.position.y, main_odom.pose.pose.position.x)
    x1 = ((robot_map_x + ((x - 160) * 0.075)) * math.cos(robot_angle))
    x2 = ((robot_map_x + ((x - 160) * 0.075)) * math.sin(robot_angle))
    y1 = ((robot_map_y + ((1 / (120 - y)) * 1800)) * math.cos(robot_angle))
    y2 = ((robot_map_y + ((-1 * (120 - y)) * 1800)) * math.sin(robot_angle))
    victim_map_x = ((robot_map_x + ((x - 160) * 0.075)) * math.cos(robot_angle))
    victim_map_y = ((robot_map_y + ((-1 * (120 - y)) * 1800)) * math.cos(robot_angle))
    return convert_from_map_to_robot(victim_map_y, victim_map_x)


def send_goal(goal_x, goal_y):
    global goal_pose, movebase_goal, movebase_client
    goal_pose = geometry_msgs.msg.PoseStamped()
    goal_pose.header.frame_id = "/map"
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    goal_pose.pose.orientation.w = 1
    movebase_goal.target_pose = goal_pose
    movebase_client.cancel_all_goals()
    movebase_client.send_goal(goal=movebase_goal, done_cb=goal_result)


def setup_movebase():
    global movebase_goal, movebase_client
    movebase_client = actionlib.SimpleActionClient('/' + namespace + '/move_base', move_base_msgs.msg.MoveBaseAction)
    movebase_client.wait_for_server()
    movebase_goal = move_base_msgs.msg.MoveBaseGoal()
    print("move base setup done")


def convert_from_robot_to_map(robot_y, robot_x):
    global main_map
    map_x = (robot_x - main_map.info.origin.position.x) // main_map.info.resolution
    map_y = (robot_y - main_map.info.origin.position.y) // main_map.info.resolution
    return map_y, map_x


def convert_from_map_to_robot(map_y, map_x):
    global main_map
    robot_x = ((map_x) * main_map.info.resolution) + main_map.info.origin.position.x
    robot_y = ((map_y) * main_map.info.resolution) + main_map.info.origin.position.y
    return robot_y, robot_x


def reset():
    global static_y, static_x
    send_goal(0, 0)


main_map = nav_msgs.msg.OccupancyGrid()
reshaped_map = np.array([])
main_odom = nav_msgs.msg.Odometry()
robot_angle = 0
namespace = 'robot0'
movebase_client = actionlib.SimpleActionClient
movebase_goal = move_base_msgs.msg.MoveBaseGoal
victim_lock = False
detected_victim_list = ()
goal_pose = geometry_msgs.msg.PoseStamped()

static_y = 0.74
static_x = 0.1

if __name__ == '__main__':

    rospy.init_node('ashkan_movebase')
    setup_movebase()
    # rospy.Subscriber('/core', nav_msgs.msg.OccupancyGrid, get_map)
    # rospy.Subscriber('/' + namespace + '/odom', nav_msgs.msg.Odometry, get_odom)
    # poses = build_victim_pose(70, 20, 0, 0)
    send_goal(0,0)
    # rospy.Subscriber('/' + namespace + '/vicims/dead', std_msgs.msg.Float64MultiArray, hot_victim)
    # rospy.spin()

#!/usr/bin/env python2

import numpy as np
import rospy
import nav_msgs.msg
import std_msgs.msg
import tf.transformations
import time
import math
import sched


def convert_from_robot_to_map(robot_y, robot_x):
    global map_info
    map_x = (robot_x - map_info.info.origin.position.x) // map_info.info.resolution
    map_y = (robot_y - map_info.info.origin.position.y) // map_info.info.resolution
    return map_y, map_x


def convert_from_map_to_robot(map_y, map_x):
    global map_info
    robot_x = (map_x * map_info.info.resolution) + map_info.info.origin.position.x
    robot_y = (map_y * map_info.info.resolution) + map_info.info.origin.position.y
    return robot_y, robot_x


def get_odom(robot_odom):
    global odom_info, robot_angle
    odom_info = robot_odom
    q = (
        robot_odom.pose.pose.orientation.x,
        robot_odom.pose.pose.orientation.y,
        robot_odom.pose.pose.orientation.z,
        robot_odom.pose.pose.orientation.w
    )
    robot_yaw = tf.transformations.euler_from_quaternion(q)
    robot_angle = robot_yaw[2] * 180 / math.pi


# codes : 1) hot victim  , 2) dead victim
def get_victim(array):
    global odom_info, rate, mark_publisher, x_data_set, y_data_set, robot_angle
    data1 = closest_pair(array.data[0], x_data_set)
    data2 = closest_pair(array.data[1], y_data_set, array.data[3])
    split1 = data1.split(' ')
    split2 = data2.split(' ')
    if math.fabs(float(split2[1]) - array.data[3]) > 4: return
    angle_from_x = float(split1[11])
    y_data_dist1 = float(split2[8][1:])
    y_data_dist2 = float(split2[10][:12])
    y_data_dist = math.sqrt(math.pow(y_data_dist1, 2) + math.pow(y_data_dist2, 2))
    r = y_data_dist / math.cos(math.radians(angle_from_x))
    theta = robot_angle - angle_from_x
    new_x1 = odom_info.pose.pose.position.x + r * math.cos(math.radians(theta))
    new_y1 = odom_info.pose.pose.position.y + r * math.sin(math.radians(theta))
    pose = convert_from_robot_to_map(new_y1, new_x1)
    add_victim(pose[1], pose[0], 1)


def closest_pair(current_data, data_set, other_data=None):
    min_score = 10000
    result = None
    if other_data is None:
        for line in data_set:
            l = line.split(' ')
            if min(min_score, abs(float(l[0]) - current_data)) != min_score:
                min_score = abs(float(l[0]) - current_data)
                result = line
    else:
        for line in data_set:
            l = line.split(' ')
            if min(min_score, abs(float(l[1]) - other_data)) != min_score:
                min_score = abs(float(l[1]) - other_data)
                result = line
    return result


def add_victim(x, y, code):
    global victim_list
    for victim_info in victim_list.keys():
        if in_range(victim_info, x, y, 0):
            victim_list[victim_info] += 1
        else:
            victim_list[(x, y, code)] = 1
    if len(victim_list) == 0:
        victim_list[(x, y, code)] = 1


def in_range(victim, x, y, tolerance=2):
    if victim[0] - tolerance <= x <= victim[0] + tolerance:
        return True
    if victim[1] - tolerance <= y <= victim[1] + tolerance:
        return True
    return False


def create_data_set(data):
    data_set = []
    data.seek(0)
    for line in data:
        data_set.append(line)
    return data_set


def check_publish_list(x, y):
    global publish_list
    check = False
    for victim in publish_list:
        if in_range(victim, x, y):
            check = True
    if not check:
        publish_list.append((x, y))
        return True
    return False


def publish_to_marker(x, y, code):
    global publish_data
    if check_publish_list(x, y):
        publish_data = std_msgs.msg.Float64MultiArray()
        publish_data.data.append(x)
        publish_data.data.append(y)
        publish_data.data.append(code)
        mark_publisher.publish(publish_data)
        rate.sleep()


def victim_tolerance():
    try:
        for victim_info in victim_list:
            if victim_list[victim_info] > 100:
                publish_to_marker(victim_info[0], victim_info[1], victim_info[2])
    except Exception:
        pass
    schedule.enter(0.5, 1, victim_tolerance, ())


if __name__ == '__main__':
    namespace = 'robot0'
    victim_list = {}
    publish_list = []
    robot_angle = 0.0
    schedule = sched.scheduler(time.time, time.sleep)
    odom_info = nav_msgs.msg.Odometry()
    rospy.init_node('victim_place')
    rate = rospy.Rate(10)
    x_data = open('../data/normalize_x_info.aura', 'r')
    y_data = open('../data/normalize_y_info.aura', 'r')
    x_data_set = create_data_set(x_data)
    y_data_set = create_data_set(y_data)
    mark_publisher = rospy.Publisher('/core/mark_place', std_msgs.msg.Float64MultiArray, queue_size=10)
    map_info = (rospy.wait_for_message('/core/map', nav_msgs.msg.OccupancyGrid))
    get_odom(rospy.wait_for_message('/' + namespace + '/odom', nav_msgs.msg.Odometry))
    rospy.Subscriber('/' + namespace + '/odom', nav_msgs.msg.Odometry, get_odom)
    rospy.Subscriber('/' + namespace + '/victims/hot', std_msgs.msg.Float64MultiArray, get_victim)
    schedule.enter(0.5, 1, victim_tolerance, ())
    schedule.run()
    rospy.spin()

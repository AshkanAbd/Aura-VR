#!/usr/bin/env python2

import rospy
import nav_msgs.msg
import std_msgs.msg
import tf.transformations
import time
import math
import sched
import os


def in_range(victim, x, y, tolerance=2):
    if victim[0] - tolerance <= x <= victim[0] + tolerance:
        return True
    if victim[1] - tolerance <= y <= victim[1] + tolerance:
        return True
    return False


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


def create_data_set(data):
    data_set = []
    data.seek(0)
    for line in data:
        data_set.append(line)
    return data_set


def generate_dataset_path():
    return os.getcwd()[0:os.getcwd().index("/", 6) + 1] + 'Aura_VR/src/aura/data/'


class HotVictimFounder:
    y_data_file = None
    x_data_file = None
    namespace = None
    y_data_set = None
    x_data_set = None
    rate = None
    robot_odom = None
    core_map = None
    mark_publisher = None
    schedule = None
    robot_angle = None
    victim_list = {}
    publish_list = []

    def __init__(self, namespace='robot0'):
        self.namespace = namespace
        self.x_data_file = open(generate_dataset_path() + 'normalize_hot_x_info.aura', 'r')
        self.y_data_file = open(generate_dataset_path() + 'normalize_hot_y_info.aura', 'r')
        self.y_data_set = create_data_set(self.y_data_file)
        self.x_data_set = create_data_set(self.x_data_file)
        self.schedule = sched.scheduler(time.time, time.sleep)
        self.schedule.enter(0.1, 1, self.victim_tolerance, ())
        self.mark_publisher = rospy.Publisher('/core/mark_place', std_msgs.msg.Float64MultiArray, queue_size=1000)
        self.core_map = rospy.wait_for_message('/core/map', nav_msgs.msg.OccupancyGrid)
        self.get_odom(rospy.wait_for_message('/' + namespace + '/odom', nav_msgs.msg.Odometry))
        rospy.Subscriber('/' + namespace + '/victims/hot', std_msgs.msg.Float64MultiArray, self.get_victim,
                         queue_size=1000)
        rospy.Subscriber('/' + namespace + '/odom', nav_msgs.msg.Odometry, self.get_odom)
        self.rate = rospy.Rate(10)
        self.schedule.run()

    def get_odom(self, odometry):
        self.robot_odom = odometry
        q = (
            odometry.pose.pose.orientation.x,
            odometry.pose.pose.orientation.y,
            odometry.pose.pose.orientation.z,
            odometry.pose.pose.orientation.w
        )
        robot_yaw = tf.transformations.euler_from_quaternion(q)
        self.robot_angle = robot_yaw[2] * 180 / math.pi

    # codes : 1) hot victim  , 2) dead victim , 3) alive victim
    def get_victim(self, array):
        data1 = closest_pair(array.data[0], self.x_data_set)
        data2 = closest_pair(array.data[1], self.y_data_set, array.data[3])
        split1 = data1.split(' ')
        split2 = data2.split(' ')
        angle_from_x = float(split1[11])
        y_data_dist1 = float(split2[8][1:])
        y_data_dist2 = float(split2[10][:12])
        y_data_dist = math.sqrt(math.pow(y_data_dist1, 2) + math.pow(y_data_dist2, 2))
        r = y_data_dist / math.cos(math.radians(angle_from_x))
        theta = self.robot_angle - angle_from_x
        new_x1 = self.robot_odom.pose.pose.position.x + r * math.cos(math.radians(theta))
        new_y1 = self.robot_odom.pose.pose.position.y + r * math.sin(math.radians(theta))
        pose = self.convert_from_robot_to_map(new_y1, new_x1)
        self.add_victim(pose[1], pose[0], 1)
        if float(split2[0]) == 0.0:
            pose = self.convert_from_robot_to_map(self.robot_odom.pose.pose.position.y,
                                                  self.robot_odom.pose.pose.position.x)
            self.add_victim(pose[1], pose[0], 1)

    def victim_tolerance(self):
        try:
            for victim_info in self.victim_list:
                if self.victim_list[victim_info] > 50:
                    self.publish_to_marker(victim_info[0], victim_info[1], victim_info[2])
        except Exception:
            pass
        self.schedule.enter(0.1, 1, self.victim_tolerance, ())

    def publish_to_marker(self, x, y, code):
        if self.check_publish_list(x, y):
            publish_data = std_msgs.msg.Float64MultiArray()
            publish_data.data.append(x)
            publish_data.data.append(y)
            publish_data.data.append(code)
            self.mark_publisher.publish(publish_data)
            self.rate.sleep()

    def check_publish_list(self, x, y):
        check = False
        for victim in self.publish_list:
            if in_range(victim, x, y):
                check = True
        if not check:
            self.publish_list.append((x, y))
            return True
        return False

    def add_victim(self, x, y, code):
        for victim_info in self.victim_list.keys():
            if in_range(victim_info, x, y, 0):
                self.victim_list[victim_info] += 1
            else:
                self.victim_list[(x, y, code)] = 1
        if len(self.victim_list) == 0:
            self.victim_list[(x, y, code)] = 1

    def convert_from_robot_to_map(self, robot_y, robot_x):
        map_x = (robot_x - self.core_map.info.origin.position.x) // self.core_map.info.resolution
        map_y = (robot_y - self.core_map.info.origin.position.y) // self.core_map.info.resolution
        return map_y, map_x

    def convert_from_map_to_robot(self, map_y, map_x):
        robot_x = (map_x * self.core_map.info.resolution) + self.core_map.info.origin.position.x
        robot_y = (map_y * self.core_map.info.resolution) + self.core_map.info.origin.position.y
        return robot_y, robot_x


class DeadVictimFounder:
    y_data_file = None
    x_data_file = None
    namespace = None
    y_data_set = None
    x_data_set = None
    rate = None
    robot_odom = None
    core_map = None
    mark_publisher = None
    schedule = None
    robot_angle = None
    victim_list = {}
    publish_list = []

    def __init__(self, namespace='robot0'):
        self.namespace = namespace
        self.x_data_file = open(generate_dataset_path() + 'normalize_dead_x_info.aura', 'r')
        self.y_data_file = open(generate_dataset_path() + 'normalize_dead_y_info.aura', 'r')
        self.y_data_set = create_data_set(self.y_data_file)
        self.x_data_set = create_data_set(self.x_data_file)
        self.schedule = sched.scheduler(time.time, time.sleep)
        self.schedule.enter(0.1, 1, self.victim_tolerance, ())
        self.mark_publisher = rospy.Publisher('/core/mark_place', std_msgs.msg.Float64MultiArray, queue_size=1000)
        self.core_map = rospy.wait_for_message('/core/map', nav_msgs.msg.OccupancyGrid)
        self.get_odom(rospy.wait_for_message('/' + namespace + '/odom', nav_msgs.msg.Odometry))
        rospy.Subscriber('/' + namespace + '/victims/dead', std_msgs.msg.Float64MultiArray, self.get_victim,
                         queue_size=1000)
        rospy.Subscriber('/' + namespace + '/odom', nav_msgs.msg.Odometry, self.get_odom)
        self.rate = rospy.Rate(10)
        self.schedule.run()

    def get_odom(self, odometry):
        self.robot_odom = odometry
        q = (
            odometry.pose.pose.orientation.x,
            odometry.pose.pose.orientation.y,
            odometry.pose.pose.orientation.z,
            odometry.pose.pose.orientation.w
        )
        robot_yaw = tf.transformations.euler_from_quaternion(q)
        self.robot_angle = robot_yaw[2] * 180 / math.pi

    # codes : 1) hot victim  , 2) dead victim , 3) alive victim
    def get_victim(self, array):
        data1 = closest_pair(array.data[0], self.x_data_set)
        data2 = closest_pair(array.data[1], self.y_data_set, array.data[3])
        split1 = data1.split(' ')
        split2 = data2.split(' ')
        angle_from_x = float(split1[11])
        y_data_dist1 = float(split2[8][1:])
        y_data_dist2 = float(split2[10][:12])
        y_data_dist = math.sqrt(math.pow(y_data_dist1, 2) + math.pow(y_data_dist2, 2))
        r = y_data_dist / math.cos(math.radians(angle_from_x))
        theta = self.robot_angle - angle_from_x
        new_x1 = self.robot_odom.pose.pose.position.x + r * math.cos(math.radians(theta))
        new_y1 = self.robot_odom.pose.pose.position.y + r * math.sin(math.radians(theta))
        pose = self.convert_from_robot_to_map(new_y1, new_x1)
        self.add_victim(pose[1], pose[0], 2)
        if float(split2[0]) == 0.0:
            pose = self.convert_from_robot_to_map(self.robot_odom.pose.pose.position.y,
                                                  self.robot_odom.pose.pose.position.x)
            self.add_victim(pose[1], pose[0], 2)

    def victim_tolerance(self):
        try:
            for victim_info in self.victim_list:
                if self.victim_list[victim_info] > 20:
                    self.publish_to_marker(victim_info[0], victim_info[1], victim_info[2])
        except Exception:
            pass
        self.schedule.enter(0.1, 1, self.victim_tolerance, ())

    def publish_to_marker(self, x, y, code):
        if self.check_publish_list(x, y):
            publish_data = std_msgs.msg.Float64MultiArray()
            publish_data.data.append(x)
            publish_data.data.append(y)
            publish_data.data.append(code)
            self.mark_publisher.publish(publish_data)
            self.rate.sleep()

    def check_publish_list(self, x, y):
        check = False
        for victim in self.publish_list:
            if in_range(victim, x, y):
                check = True
        if not check:
            self.publish_list.append((x, y))
            return True
        return False

    def add_victim(self, x, y, code):
        for victim_info in self.victim_list.keys():
            if in_range(victim_info, x, y, 0):
                self.victim_list[victim_info] += 1
            else:
                self.victim_list[(x, y, code)] = 1
        if len(self.victim_list) == 0:
            self.victim_list[(x, y, code)] = 1

    def convert_from_robot_to_map(self, robot_y, robot_x):
        map_x = (robot_x - self.core_map.info.origin.position.x) // self.core_map.info.resolution
        map_y = (robot_y - self.core_map.info.origin.position.y) // self.core_map.info.resolution
        return map_y, map_x

    def convert_from_map_to_robot(self, map_y, map_x):
        robot_x = (map_x * self.core_map.info.resolution) + self.core_map.info.origin.position.x
        robot_y = (map_y * self.core_map.info.resolution) + self.core_map.info.origin.position.y
        return robot_y, robot_x


class AliveVictimFounder:
    y_data_file = None
    x_data_file = None
    namespace = None
    y_data_set = None
    x_data_set = None
    rate = None
    robot_odom = None
    core_map = None
    mark_publisher = None
    schedule = None
    robot_angle = None
    victim_list = {}
    publish_list = []

    def __init__(self, namespace='robot0'):
        self.namespace = namespace
        self.x_data_file = open(generate_dataset_path() + 'normalize_hot_x_info.aura', 'r')
        self.y_data_file = open(generate_dataset_path() + 'normalize_hot_y_info.aura', 'r')
        self.y_data_set = create_data_set(self.y_data_file)
        self.x_data_set = create_data_set(self.x_data_file)
        self.schedule = sched.scheduler(time.time, time.sleep)
        self.schedule.enter(0.1, 1, self.victim_tolerance, ())
        self.mark_publisher = rospy.Publisher('/core/mark_place', std_msgs.msg.Float64MultiArray, queue_size=1000)
        self.core_map = rospy.wait_for_message('/core/map', nav_msgs.msg.OccupancyGrid)
        self.get_odom(rospy.wait_for_message('/' + namespace + '/odom', nav_msgs.msg.Odometry))
        rospy.Subscriber('/' + namespace + '/victims/alive', std_msgs.msg.Float64MultiArray, self.get_victim,
                         queue_size=1000)
        rospy.Subscriber('/' + namespace + '/odom', nav_msgs.msg.Odometry, self.get_odom)
        self.rate = rospy.Rate(10)
        self.schedule.run()

    def get_odom(self, odometry):
        self.robot_odom = odometry
        q = (
            odometry.pose.pose.orientation.x,
            odometry.pose.pose.orientation.y,
            odometry.pose.pose.orientation.z,
            odometry.pose.pose.orientation.w
        )
        robot_yaw = tf.transformations.euler_from_quaternion(q)
        self.robot_angle = robot_yaw[2] * 180 / math.pi

    # codes : 1) hot victim  , 2) dead victim , 3) alive victim
    def get_victim(self, array):
        data1 = closest_pair(array.data[0], self.x_data_set)
        data2 = closest_pair(array.data[1], self.y_data_set, array.data[3])
        split1 = data1.split(' ')
        split2 = data2.split(' ')
        angle_from_x = float(split1[11])
        y_data_dist1 = float(split2[8][1:])
        y_data_dist2 = float(split2[10][:12])
        y_data_dist = math.sqrt(math.pow(y_data_dist1, 2) + math.pow(y_data_dist2, 2))
        r = y_data_dist / math.cos(math.radians(angle_from_x))
        theta = self.robot_angle - angle_from_x
        new_x1 = self.robot_odom.pose.pose.position.x + r * math.cos(math.radians(theta))
        new_y1 = self.robot_odom.pose.pose.position.y + r * math.sin(math.radians(theta))
        pose = self.convert_from_robot_to_map(new_y1, new_x1)
        self.add_victim(pose[1], pose[0], 3)
        if float(split2[0]) == 0.0:
            pose = self.convert_from_robot_to_map(self.robot_odom.pose.pose.position.y,
                                                  self.robot_odom.pose.pose.position.x)
            self.add_victim(pose[1], pose[0], 3)

    def victim_tolerance(self):
        try:
            for victim_info in self.victim_list:
                if self.victim_list[victim_info] > 50:
                    self.publish_to_marker(victim_info[0], victim_info[1], victim_info[2])
        except Exception:
            pass
        self.schedule.enter(0.1, 1, self.victim_tolerance, ())

    def publish_to_marker(self, x, y, code):
        if self.check_publish_list(x, y):
            publish_data = std_msgs.msg.Float64MultiArray()
            publish_data.data.append(x)
            publish_data.data.append(y)
            publish_data.data.append(code)
            self.mark_publisher.publish(publish_data)
            self.rate.sleep()

    def check_publish_list(self, x, y):
        check = False
        for victim in self.publish_list:
            if in_range(victim, x, y):
                check = True
        if not check:
            self.publish_list.append((x, y))
            return True
        return False

    def add_victim(self, x, y, code):
        for victim_info in self.victim_list.keys():
            if in_range(victim_info, x, y, 0):
                self.victim_list[victim_info] += 1
            else:
                self.victim_list[(x, y, code)] = 1
        if len(self.victim_list) == 0:
            self.victim_list[(x, y, code)] = 1

    def convert_from_robot_to_map(self, robot_y, robot_x):
        map_x = (robot_x - self.core_map.info.origin.position.x) // self.core_map.info.resolution
        map_y = (robot_y - self.core_map.info.origin.position.y) // self.core_map.info.resolution
        return map_y, map_x

    def convert_from_map_to_robot(self, map_y, map_x):
        robot_x = (map_x * self.core_map.info.resolution) + self.core_map.info.origin.position.x
        robot_y = (map_y * self.core_map.info.resolution) + self.core_map.info.origin.position.y
        return robot_y, robot_x

#!/usr/bin/env python2.7

import rospy
import aura.msg
import nav_msgs.msg
import sys
import os
import cv2 as cv
import numpy as np


class VictimVerifier:
    core_map = None
    namespace = None
    reshaped_map = None
    publish_to_auto_move = None

    def __init__(self, namespace):
        self.namespace = namespace
        self.publish_to_auto_move = rospy.Publisher('/' + namespace + '/goto_victim', aura.msg.data_float,
                                                    queue_size=1000)
        self.get_core_map(rospy.wait_for_message('/core/map', nav_msgs.msg.OccupancyGrid))

    # CODES: 1) mark pose , 2) go to pose , 3) mark here
    def check_victim(self, data_line, victim_info, pose):
        self.get_core_map(rospy.wait_for_message('/core/map', nav_msgs.msg.OccupancyGrid))
        odom = rospy.wait_for_message('/' + self.namespace + '/odom', nav_msgs.msg.Odometry)
        sub_map = self.reshaped_map[int(pose[1] - 2):int(pose[1] + 3), int(pose[0] - 2):int(pose[0] + 3)]
        one_count = len(np.where(sub_map == 100)[0])
        map_y, map_x = self.convert_from_robot_to_map(odom.pose.pose.position.y, odom.pose.pose.position.x)
        sub_map1 = self.reshaped_map[int(map_y - 2):int(map_y + 3), int(map_x - 2):int(map_y + 3)]
        one_count1 = len(np.where(sub_map1 == 100)[0])
        if abs(float(data_line[0]) - float(victim_info[1])) > 2:
            return 2
        if float(data_line[0]) == 0.0:
            if 0 < one_count1 <= 2:
                return 3
        # print (one_count)
        if 0 < one_count <= 2:
            return 1
        return 4

    def publish_pose(self, code, x, y):
        data = aura.msg.data_float()
        data.data_float.append(code)
        data.data_float.append(x)
        data.data_float.append(y)
        self.publish_to_auto_move.publish(data)

    def get_core_map(self, core):
        self.core_map = core
        self.reshaped_map = np.asarray(core.data, dtype=np.int8).reshape((core.info.height, core.info.width))

    def convert_from_robot_to_map(self, robot_y, robot_x):
        map_x = round((robot_x - self.core_map.info.origin.position.x) / self.core_map.info.resolution)
        map_y = round((robot_y - self.core_map.info.origin.position.y) / self.core_map.info.resolution)
        return map_y, map_x

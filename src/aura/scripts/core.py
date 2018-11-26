#!/usr/bin/env python3

import rospy
import nav_msgs.msg
import numpy as np
import sys
import message_filters


class CoreMapBuilder:
    robot0 = 'robot0'
    robot1 = 'robot1'
    robot2 = 'robot2'
    robot3 = 'robot3'
    core_map = np.array([])
    # tolerance_zero = {}
    # tolerance_one = {}
    robot_evolution = {}
    node_distance = {}
    close_list = set()
    main_map = None
    core_publisher = None
    rate = None

    def __init__(self, robot0='robot0', robot1='robot1', robot2='robot2', robot3='robot3'):
        self.robot0 = robot0
        self.robot1 = robot1
        self.robot2 = robot2
        self.robot3 = robot3
        rospy.init_node('core_builder')
        # rospy.Subscriber('/' + self.robot0 + '/map', nav_msgs.msg.OccupancyGrid, self.get_robots_map, '1')
        # rospy.Subscriber('/' + self.robot1 + '/map', nav_msgs.msg.OccupancyGrid, self.get_robots_map, '2')
        # rospy.Subscriber('/' + self.robot2 + '/map', nav_msgs.msg.OccupancyGrid, self.get_robots_map, '3')
        # rospy.Subscriber('/' + self.robot3 + '/map', nav_msgs.msg.OccupancyGrid, self.get_robots_map, '4')
        map_sub0 = message_filters.Subscriber('/' + self.robot0 + '/map', nav_msgs.msg.OccupancyGrid)
        map_sub1 = message_filters.Subscriber('/' + self.robot1 + '/map', nav_msgs.msg.OccupancyGrid)
        map_sub2 = message_filters.Subscriber('/' + self.robot2 + '/map', nav_msgs.msg.OccupancyGrid)
        map_sub3 = message_filters.Subscriber('/' + self.robot3 + '/map', nav_msgs.msg.OccupancyGrid)
        odom_sub0 = message_filters.Subscriber('/' + self.robot0 + '/odom', nav_msgs.msg.Odometry)
        odom_sub1 = message_filters.Subscriber('/' + self.robot1 + '/odom', nav_msgs.msg.Odometry)
        odom_sub2 = message_filters.Subscriber('/' + self.robot2 + '/odom', nav_msgs.msg.Odometry)
        odom_sub3 = message_filters.Subscriber('/' + self.robot3 + '/odom', nav_msgs.msg.Odometry)
        sync0 = message_filters.TimeSynchronizer((map_sub0, odom_sub0), 1000)
        sync1 = message_filters.TimeSynchronizer((map_sub1, odom_sub1), 1000)
        sync2 = message_filters.TimeSynchronizer((map_sub2, odom_sub2), 1000)
        sync3 = message_filters.TimeSynchronizer((map_sub3, odom_sub3), 1000)
        sync0.registerCallback(self.get_robots_map)
        sync1.registerCallback(self.get_robots_map)
        sync2.registerCallback(self.get_robots_map)
        sync3.registerCallback(self.get_robots_map)
        self.core_publisher = rospy.Publisher('/core/map', nav_msgs.msg.OccupancyGrid, queue_size=100)
        self.rate = rospy.Rate(5)
        self.publish_to_core()

    def get_robots_map(self, robot_map, odom):
        self.main_map = robot_map
        if self.robot0 in odom.child_frame_id:
            robot_id = self.robot0
        elif self.robot1 in odom.child_frame_id:
            robot_id = self.robot1
        elif self.robot2 in odom.child_frame_id:
            robot_id = self.robot2
        else:
            robot_id = self.robot3
        self.build_core_map(robot_map, odom, robot_id)

    def publish_to_core(self):
        while not rospy.is_shutdown():
            if self.main_map is None or self.core_map.shape[0] == 0:
                continue
            # try:
            data_map = self.main_map
            data_map.data = self.core_map.tolist()
            data_map.header.stamp = rospy.Time.now()
            self.core_publisher.publish(data_map)
            self.rate.sleep()
            # except Exception as e:
            #     print(e)

    def build_core_map(self, robot_map, odom, robot_id):
        map1 = np.asarray(robot_map.data)
        if self.core_map.shape != map1.shape:
            self.core_map = np.zeros(map1.shape, map1.dtype)
            self.core_map = self.core_map - 1
        new_zero_coo = np.where(map1 == 0)[0]
        new_one_coo = np.where(map1 == 100)[0]
        for coordinate in new_zero_coo:
            if self.core_map[coordinate] == -1:
                self.core_map[coordinate] = 0
                self.robot_evolution[coordinate] = robot_id
                self.node_distance[coordinate] = (robot_id, self.convert_from_robot_to_map(odom.pose.pose.position.y
                                                                                           , odom.pose.pose.position.x))
            elif (self.core_map[coordinate] == 100) and (coordinate not in self.close_list):
                self.close_list.add(coordinate)
                if self.robot_evolution[coordinate] == robot_id:
                    self.core_map[coordinate] = 0
                    self.node_distance[coordinate] = (robot_id,
                                                      self.convert_from_robot_to_map(odom.pose.pose.position.y
                                                                                     , odom.pose.pose.position.x))
                else:
                    distance = self.convert_from_robot_to_map(odom.pose.pose.position.y, odom.pose.pose.position.x)
                    if self.node_distance[coordinate][1] > distance:
                        self.core_map[coordinate] = 0
                        self.node_distance[coordinate] = (robot_id,
                                                          self.convert_from_robot_to_map(odom.pose.pose.position.y
                                                                                         , odom.pose.pose.position.x))
                    # if coordinate not in self.tolerance_zero:
                    #     self.tolerance_zero[coordinate] = 1
                    # else:
                    #     self.tolerance_zero[coordinate] += 1
        for coordinate in new_one_coo:
            if self.core_map[coordinate] == -1:
                self.core_map[coordinate] = 100
                self.robot_evolution[coordinate] = robot_id
                self.node_distance[coordinate] = (robot_id, self.convert_from_robot_to_map(odom.pose.pose.position.y,
                                                                                           odom.pose.pose.position.x))
            elif (self.core_map[coordinate] == 0) and (coordinate not in self.close_list):
                self.close_list.add(coordinate)
                if self.robot_evolution[coordinate] == robot_id:
                    self.core_map[coordinate] = 100
                    self.node_distance[coordinate] = (robot_id,
                                                      self.convert_from_robot_to_map(odom.pose.pose.position.y,
                                                                                     odom.pose.pose.position.x))
                else:
                    distance = self.convert_from_robot_to_map(odom.pose.pose.position.y, odom.pose.pose.position.x)
                    if self.node_distance[coordinate][1] > distance:
                        self.node_distance[coordinate] = (robot_id,
                                                          self.convert_from_robot_to_map(odom.pose.pose.position.y,
                                                                                         odom.pose.pose.position.x))

                    # if coordinate not in self.tolerance_one:
                    #     self.tolerance_one[coordinate] = 1
                    # else:
                    #     self.tolerance_one[coordinate] += 1
        # end = []
        # for pair in self.tolerance_one.keys():
        #     if self.tolerance_one[pair] > 10 and self.core_map[pair[0], pair[1]] == 0:
        #         self.core_map[pair[0], pair[1]] = 100
        #         end.append(pair)
        # for pair in self.tolerance_zero.keys():
        #     if self.tolerance_zero[pair] > 10 and self.core_map[pair[0], pair[1]] == 100:
        #         end.append(pair)
        #         self.core_map[pair[0], pair[1]] = 0
        # for pair in end:
        #     if pair in self.tolerance_zero:
        #         self.tolerance_zero.pop(pair)
        #     if pair in self.tolerance_one:
        #         self.tolerance_one.pop(pair)

    def convert_from_robot_to_map(self, robot_y, robot_x):
        map_x = (robot_x - self.main_map.info.origin.position.x) // self.main_map.info.resolution
        map_y = (robot_y - self.main_map.info.origin.position.y) // self.main_map.info.resolution
        return (map_y * self.main_map.info.width) + map_x


if __name__ == '__main__':
    core_map_builder = None
    if len(sys.argv) > 5:
        _robot1 = sys.argv[1]
        _robot2 = sys.argv[2]
        _robot3 = sys.argv[3]
        _robot4 = sys.argv[4]
        core_map_builder = CoreMapBuilder(_robot1, _robot2, _robot3, _robot4)
    else:
        core_map_builder = CoreMapBuilder()
    rospy.spin()

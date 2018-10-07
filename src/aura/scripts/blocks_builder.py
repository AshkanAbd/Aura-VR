#!/usr/bin/env python3

import rospy
import numpy as np
import nav_msgs.msg
import std_msgs.msg
import aura.msg

map = nav_msgs.msg.OccupancyGrid
map_info = nav_msgs.msg.OccupancyGrid.info


def divide():
    global map_info, map_width, map_height
    group = aura.msg.group()
    block_size = map_width // 16
    for i in range(map_height // block_size):
        for j in range(map_width // block_size):
            data = aura.msg.data()
            temp = map_info[i * block_size:(i + 1) * block_size, j * block_size:(j + 1) * block_size].copy()
            temp = temp.reshape(3844).tolist()
            data.data = temp
            group.array.append(data)
    return group


def get_map(map: nav_msgs.msg.OccupancyGrid):
    global map_width, map_height, map_info, cluster_publisher
    map_info = map
    map_x = map.info.width
    map_y = map.info.height
    map_info = np.asarray(map.data)
    map_info = map_info.reshape(map_y, map_x)
    result = divide()
    cluster_publisher.publish(result)


if __name__ == '__main__':
    map_info = None
    map_width = None
    map_height = None
    rospy.init_node('blocks_builder')
    cluster_publisher = rospy.Publisher('/core/blocks', aura.msg.group, queue_size=20)
    rospy.Subscriber('/core/map', nav_msgs.msg.OccupancyGrid, get_map)
    rospy.spin()

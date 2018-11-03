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
    group = aura.msg.group_int()
    block_size_col = map_width // 16
    block_size_row = map_height // 16
    for i in range(map_height // block_size_row):
        for j in range(map_width // block_size_col):
            data = aura.msg.data_int()
            temp = map_info[i * block_size_row:(i + 1) * block_size_row,
                   j * block_size_col:(j + 1) * block_size_row].copy()
            temp = temp.reshape(block_size_row * block_size_col).tolist()
            data.data_int = temp
            group.array.append(data)
    return group


def get_map(map: nav_msgs.msg.OccupancyGrid):
    global map_width, map_height, map_info, cluster_publisher
    map_info = map
    map_width = map.info.width
    map_height = map.info.height
    map_info = np.asarray(map.data)
    map_info = map_info.reshape(map_height, map_width)
    result = divide()
    cluster_publisher.publish(result)


if __name__ == '__main__':
    map_info = None
    map_width = None
    map_height = None
    rospy.init_node('blocks_builder')
    cluster_publisher = rospy.Publisher('/core/blocks', aura.msg.group_int, queue_size=20)
    rospy.Subscriber('/core/map', nav_msgs.msg.OccupancyGrid, get_map)
    rospy.spin()

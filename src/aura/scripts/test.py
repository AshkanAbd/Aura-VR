#!/usr/bin/env python3

import rospy
import numpy as np
import aura.msg
import nav_msgs.msg


def get_block(blocks: aura.msg.group_int):
    global publish
    map = nav_msgs.msg.OccupancyGrid()
    index = 137
    map.data = blocks.array[index].data_int
    map.info.width = 992 // 16
    map.info.height = 992 // 16
    map.info.resolution = 0.2
    start_x = (index % 16) - 6.45
    start_y = (index // 16) - 6.45
    map.info.origin.position.x = ((start_x * 62) - 100) * 0.2
    map.info.origin.position.y = ((start_y * 62) - 100) * 0.2
    publish.publish(map)


if __name__ == '__main__':
    namespace = 'robot0'
    rospy.init_node('test')
    rospy.Subscriber('/core/blocks', aura.msg.group_int, get_block)
    publish = rospy.Publisher('/test', nav_msgs.msg.OccupancyGrid, queue_size=10000)
    rospy.spin()

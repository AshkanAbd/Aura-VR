#!/usr/bin/env python3

import rospy
import numpy as np
import aura.msg
import nav_msgs.msg


def get_block(blocks: aura.msg.group_int):
    global publish
    map = nav_msgs.msg.OccupancyGrid()
    map.data = blocks.array[136].data_int
    map.info.width = 30
    map.info.height = 30
    map.info.resolution = 0.2
    publish.publish(map)


if __name__ == '__main__':
    namespace = 'robot0'
    rospy.init_node('test')
    rospy.Subscriber('/core/blocks', aura.msg.group_int, get_block)
    publish = rospy.Publisher('/test', nav_msgs.msg.OccupancyGrid, queue_size=10)
    rospy.spin()

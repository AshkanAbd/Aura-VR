#!/usr/bin/env python3

import rospy
import single_Bfs

if __name__ == '__main__':
    bfs_move_base = single_Bfs.SingleBFS()
    bfs_move_base.start()
    rospy.spin()
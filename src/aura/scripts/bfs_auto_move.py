#!/usr/bin/env python2

import rospy
import DFSAutoMove
import sys

if __name__ == '__main__':
    dfs_move_base = DFSAutoMove.DFSAutoMove(sys.argv[1])
    dfs_move_base.bfsihdir(dfs_move_base.robot_block)
    rospy.spin()

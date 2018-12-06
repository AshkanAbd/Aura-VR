#!/usr/bin/env python2

import rospy
import BFSAutoMove
import sys

if __name__ == '__main__':
    dfs_move_base = BFSAutoMove.BFSAutoMove(sys.argv[1])
    dfs_move_base.start()
    rospy.spin()

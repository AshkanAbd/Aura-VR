#!/usr/bin/env python2

import rospy
import auto_move_base
import aura.msg
import block
import DFSAutoMove
import marker
import geometry_msgs.msg

# def get_current_goal(current_goal: geometry_msgs.msg.PoseStamped):
#     global marker1
#     marker1.create_and_add_marker(0, 0, 255, current_goal.pose.position.x, current_goal.pose.position.y)


if __name__ == '__main__':
    # marker1 = marker.MarkerController('robot0')
    dfs_move_base = DFSAutoMove.DFSAutoMove('aura1')
    dfs_move_base.bfsihdir(dfs_move_base.robot_block)
    # rospy.Subscriber('/robot0/move_base/current_goal', geometry_msgs.msg.PoseStamped, get_current_goal)
    rospy.spin()

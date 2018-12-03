#!/usr/bin/env python2

import rospy
import victim_founder
import sys

if __name__ == '__main__':
    rospy.init_node('dead_victim_place')
    victim_founder.DeadVictimFounder(sys.argv[1])
    rospy.spin()
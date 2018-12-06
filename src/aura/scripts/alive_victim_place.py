#!/usr/bin/env python2

import rospy
import victim_founder
import sys

if __name__ == '__main__':
    rospy.init_node('alive_victim_place')
    victim_founder.AliveVictimFounder(sys.argv[1])
    rospy.spin()
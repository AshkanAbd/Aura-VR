#!/usr/bin/env python2

import rospy
import sys
import victim_founder

if __name__ == '__main__':
    rospy.init_node('hot_victim_place')
    hot_founder = victim_founder.HotVictimFounder(sys.argv[1])
    rospy.spin()

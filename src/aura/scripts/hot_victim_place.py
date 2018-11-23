#!/usr/bin/env python2

import rospy
import victim_founder

if __name__ == '__main__':
    rospy.init_node('a')
    hot_founder = victim_founder.HotVictimFounder()
    rospy.spin()

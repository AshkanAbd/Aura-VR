#!/usr/bin/env python2

import rospy
import victim_founder

if __name__ == '__main__':
    rospy.init_node('ab')
    victim_founder.DeadVictimFounder()
    rospy.spin()
#!/usr/bin/env python2.7

import rospy
import aura.msg
import nav_msgs.msg
import sys
import os
import cv2 as cv


class VerifyVictims:
    victims = set()

    def __init__(self):
        pass

    def check(self, x, y):
        core_map = rospy.wait_for_message('/core/map', nav_msgs.msg.OccupancyGrid)

        pass

    def add(self):
        pass

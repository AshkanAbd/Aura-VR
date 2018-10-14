#!/usr/bin/env python3

import rospy
import std_msgs.msg
import nav_msgs.msg
import numpy as np
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *


def convert_from_robot_to_map(robot_y, robot_x):
    global map_info
    map_x = (robot_x - map_info.info.origin.position.x) // map_info.info.resolution
    map_y = (robot_y - map_info.info.origin.position.y) // map_info.info.resolution
    return map_y, map_x


def convert_from_map_to_robot(map_y, map_x):
    global map_info
    robot_x = (map_x * map_info.info.resolution) + map_info.info.origin.position.x
    robot_y = (map_y * map_info.info.resolution) + map_info.info.origin.position.y
    return robot_y, robot_x


def process_feedback(feedback):
    p = feedback.pose.position
    print(feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z))


def create_marker(r, g, b, x=0, y=0):
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.85
    box_marker.scale.y = 0.85
    box_marker.scale.z = 0.85
    box_marker.color.r = r
    box_marker.color.g = g
    box_marker.color.b = b
    box_marker.color.a = 1.0
    box_marker.pose.position.x = x
    box_marker.pose.position.y = y
    return box_marker


def add_marker(marker):
    global box_control
    box_control.markers.append(marker)


def get_mark_place(place: std_msgs.msg.Float64MultiArray):
    pose = convert_from_map_to_robot(place.data[1], place.data[0])
    if check_map(place.data[0], place.data[1]):
        print("Noise")
        return
    print("Received")
    if place.data[2] == 1:
        marker = create_marker(255, 0, 0, pose[1], pose[0])
        add_marker(marker)
    else:
        marker = create_marker(0, 0, 255, pose[1], pose[0])
        add_marker(marker)


def check_map(x, y):
    reshape_map = np.asarray(map_info.data).reshape(map_info.info.height, map_info.info.width)
    if reshape_map[int(y), int(x)] == -1:
        return True
    return False


def get_map(core: nav_msgs.msg.OccupancyGrid):
    global map_info
    map_info = core


def main():
    global box_control, map_info
    rospy.init_node("victim_marker")
    server = InteractiveMarkerServer("/" + namespace + "/victim_marker")
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/map"
    int_marker.name = "victim_marker"
    int_marker.description = "victim marker"
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    int_marker.controls.append(box_control)
    server.insert(int_marker, process_feedback)
    server.applyChanges()
    map_info = (rospy.wait_for_message("/core/map", nav_msgs.msg.OccupancyGrid))
    rospy.Subscriber('/core/map', nav_msgs.msg.OccupancyGrid, get_map)
    rospy.Subscriber('/core/mark_place', std_msgs.msg.Float64MultiArray, get_mark_place)
    rospy.spin()


if __name__ == "__main__":
    namespace = 'robot0'
    box_control = None
    map_info = nav_msgs.msg.OccupancyGrid()
    main()

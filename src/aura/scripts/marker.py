#!/usr/bin/env python3

import rospy
import std_msgs.msg
import nav_msgs.msg
import numpy as np
import visualization_msgs.msg
import interactive_markers.interactive_marker_server

class MarkerController:
    box_control = None

    def __init__(self, namespace):
        server = interactive_markers.interactive_marker_server.InteractiveMarkerServer(
            "/" + namespace + "/victim_marker")
        int_marker = interactive_markers.interactive_marker_server.InteractiveMarker()
        int_marker.header.frame_id = "/map"
        int_marker.name = "victim_marker"
        int_marker.description = "victim marker"
        self.box_control = visualization_msgs.msg.InteractiveMarkerControl()
        self.box_control.always_visible = True
        int_marker.controls.append(self.box_control)
        server.insert(int_marker)
        server.applyChanges()

    def create_and_add_marker(self, r, g, b, x=0.0, y=0.0):
        box_marker = visualization_msgs.msg.Marker()
        box_marker.type = visualization_msgs.msg.Marker.CUBE
        box_marker.scale.x = 0.85
        box_marker.scale.y = 0.85
        box_marker.scale.z = 0.85
        box_marker.color.r = r
        box_marker.color.g = g
        box_marker.color.b = b
        box_marker.color.a = 1.0
        box_marker.pose.position.x = x
        box_marker.pose.position.y = y
        self.box_control.markers.append(box_marker)


def get_mark_place(place):
    global marker_controller
    pose = convert_from_map_to_robot(place.data[1], place.data[0])
    if verify_data(place.data[0], place.data[1]):
        print("Noise")
        return
    if place.data[2] == 1:
        print("HOT Received")
        marker_controller.create_and_add_marker(255, 0, 0, pose[1], pose[0])
    else:
        print("DEAD Received")
        marker_controller.create_and_add_marker(0, 0, 255, pose[1], pose[0])


def verify_data(x, y):
    reshape_map = np.asarray(map_info.data).reshape(map_info.info.height, map_info.info.width)
    if reshape_map[int(y), int(x)] == -1:
        return True
    return False


def get_map(core_map):
    global map_info
    map_info = core_map


def convert_from_robot_to_map(robot_y, robot_x):
    map_x = (robot_x - map_info.info.origin.position.x) // map_info.info.resolution
    map_y = (robot_y - map_info.info.origin.position.y) // map_info.info.resolution
    return map_y, map_x


def convert_from_map_to_robot(map_y, map_x):
    robot_x = (map_x * map_info.info.resolution) + map_info.info.origin.position.x
    robot_y = (map_y * map_info.info.resolution) + map_info.info.origin.position.y
    return robot_y, robot_x


def main():
    global map_info, marker_controller
    rospy.init_node("victim_marker")
    rospy.Subscriber('/core/map', nav_msgs.msg.OccupancyGrid, get_map)
    rospy.Subscriber('/core/mark_place', std_msgs.msg.Float64MultiArray, get_mark_place)
    marker_controller = MarkerController('robot0')
    marker_controller.create_and_add_marker(255, 0, 0,-34.7999990284 , 19.4000017792)
    rospy.spin()


if __name__ == "__main__":
    map_info = nav_msgs.msg.OccupancyGrid()
    marker_controller = None
    main()

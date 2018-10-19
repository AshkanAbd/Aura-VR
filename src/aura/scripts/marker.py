#!/usr/bin/env python3

import rospy
import std_msgs.msg
import nav_msgs.msg
import numpy as np
import visualization_msgs.msg
import interactive_markers.interactive_marker_server
import core


class MarkerController:
    box_control = None

    def __init__(self, namespace):
        server = interactive_markers.interactive_marker_server.InteractiveMarkerServer(
            "/" + namespace + "/victim_marker")
        int_marker = interactive_markers.interactive_marker_server.InteractiveMarker()
        int_marker.header.frame_id = "/map"
        int_marker.name = "victim_marker"
        int_marker.description = "victim marker"
        box_control = visualization_msgs.msg.InteractiveMarkerControl()
        box_control.always_visible = True
        int_marker.controls.append(box_control)
        server.insert(int_marker)
        server.applyChanges()

    def create_and_add_marker(self, r, g, b, x=0, y=0):
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


def get_mark_place(place: std_msgs.msg.Float64MultiArray):
    global marker_controller
    pose = core.convert_from_map_to_robot(place.data[1], place.data[0])
    if verify_data(place.data[0], place.data[1]):
        print("Noise")
        return
    print("Received")
    if place.data[2] == 1:
        marker_controller.create_and_add_marker(255, 0, 0, pose[1], pose[0])
    else:
        marker_controller.create_and_add_marker(0, 0, 255, pose[1], pose[0])


def verify_data(x, y):
    reshape_map = np.asarray(map_info.data).reshape(map_info.info.height, map_info.info.width)
    if reshape_map[int(y), int(x)] == -1:
        return True
    return False


def get_map(core_map):
    global map_info
    map_info = core_map


def main():
    global map_info
    rospy.init_node("victim_marker")
    rospy.Subscriber('/core/map', nav_msgs.msg.OccupancyGrid, get_map)
    rospy.Subscriber('/core/mark_place', std_msgs.msg.Float64MultiArray, get_mark_place)
    rospy.spin()


if __name__ == "__main__":
    map_info = nav_msgs.msg.OccupancyGrid()
    marker_controller = MarkerController('robot0')
    main()

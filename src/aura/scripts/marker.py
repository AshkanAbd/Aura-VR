#!/usr/bin/env python2

import rospy
import std_msgs.msg
import nav_msgs.msg
import numpy as np
import visualization_msgs.msg
import interactive_markers.interactive_marker_server


class MarkerController:
    box_control = None

    def __init__(self):
        server = interactive_markers.interactive_marker_server.InteractiveMarkerServer(
            "/core/victim_marker")
        int_marker = interactive_markers.interactive_marker_server.InteractiveMarker()
        int_marker.header.frame_id = "/map"
        int_marker.name = "victim_marker"
        int_marker.description = "victim marker"
        self.box_control = visualization_msgs.msg.InteractiveMarkerControl()
        self.box_control.always_visible = True
        int_marker.controls.append(self.box_control)
        server.insert(int_marker)
        server.applyChanges()

    def add_marker(self, box_marker):
        self.box_control.markers.append(box_marker)

    def create_marker(self, r, g, b, x=0.0, y=0.0):
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
        return box_marker


class Marker:
    map_info = nav_msgs.msg.OccupancyGrid()
    marker_controller = None
    victim_set = {}
    marker_set = {}

    def __init__(self):
        rospy.init_node("victim_marker")
        rospy.Subscriber('/core/map', nav_msgs.msg.OccupancyGrid, self.get_map)
        rospy.Subscriber('/core/mark_place', std_msgs.msg.Float64MultiArray, self.get_mark_place)
        self.marker_controller = MarkerController()

    def get_mark_place(self, place):
        pose = self.convert_from_map_to_robot(place.data[1], place.data[0])
        if self.verify_data(place.data[0], place.data[1], place.data[2]):
            print("Noise")
            return
        if place.data[2] == 1:
            print("HOT Received")
            marker = self.marker_controller.create_marker(255, 0, 0, pose[1], pose[0])
            self.marker_set[(place.data[0], place.data[1])] = marker
            self.marker_controller.add_marker(marker)
        elif place.data[2] == 2:
            print("DEAD Received")
            marker = self.marker_controller.create_marker(0, 0, 255, pose[1], pose[0])
            self.marker_set[(place.data[0], place.data[1])] = marker
            self.marker_controller.add_marker(marker)
        elif place.data[2] == 3:
            print("ALIVE Received")
            marker = self.marker_controller.create_marker(0, 255, 0, pose[1], pose[0])
            self.marker_set[(place.data[0], place.data[1])] = marker
            self.marker_controller.add_marker(marker)

    def convert_from_robot_to_map(self, robot_y, robot_x):
        map_x = round((robot_x - self.map_info.info.origin.position.x) / self.map_info.info.resolution)
        map_y = round((robot_y - self.map_info.info.origin.position.y) / self.map_info.info.resolution)
        return map_y, map_x

    def convert_from_map_to_robot(self, map_y, map_x):
        robot_x = (map_x * self.map_info.info.resolution) + self.map_info.info.origin.position.x
        robot_y = (map_y * self.map_info.info.resolution) + self.map_info.info.origin.position.y
        return robot_y, robot_x

    def verify_data(self, x, y, code):
        # reshape_map = np.asarray(self.map_info.data).reshape(self.map_info.info.height, self.map_info.info.width)
        for pair in self.victim_set.keys():
            tmp = []
            for i in xrange(-3, 4):
                for j in xrange(-3, 4):
                    tmp.append((pair[0] + i, pair[1] + j))
            if (x, y) in tmp:
                if self.victim_set[pair] == 3 and code == 1:
                    self.victim_set.pop(pair)
                    self.victim_set[(x, y)] = code
                    self.marker_set[(x, y)].color.g = 0
                    self.marker_set[(x, y)].color.r = 255
                    return False
                return True
        self.victim_set[(x, y)] = code
        return False

    def get_map(self, core_map):
        self.map_info = core_map


if __name__ == "__main__":
    Marker()
    rospy.spin()

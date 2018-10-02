import rospy
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *


def process_feedback(feedback):
    p = feedback.pose.position
    print(feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z))


def create_marker(r, g, b, x=0, y=0):
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.45
    box_marker.scale.z = 0.45
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


namespace = 'robot0'
box_control = None


def main():
    global box_control
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
    rospy.spin()


if __name__ == "__main__":
    main()

import rospy
import nav_msgs.msg
import std_msgs.msg
import tf.transformations
import math
import numpy as np


def convert_from_map_to_robot(map_y, map_x):
    global core_map
    robot_x = (map_x * core_map.info.resolution) + core_map.info.origin.position.x
    robot_y = (map_y * core_map.info.resolution) + core_map.info.origin.position.y
    return robot_y, robot_x


def convert_from_robot_to_map(robot_y, robot_x):
    global core_map
    map_x = (robot_x - core_map.info.origin.position.x) // core_map.info.resolution
    map_y = (robot_y - core_map.info.origin.position.y) // core_map.info.resolution
    return map_y, map_x


def get_odom(odom):
    global robot_odom
    robot_odom = odom


def get_core(core):
    global core_map
    core_map = core


# victim in (250 , 300)
def get_victim(array):
    global victim_info, y_list, file, robot_odom
    victim_info = array
    q = (
        robot_odom.pose.pose.orientation.x,
        robot_odom.pose.pose.orientation.y,
        robot_odom.pose.pose.orientation.z,
        robot_odom.pose.pose.orientation.w
    )
    robot_yaw = tf.transformations.euler_from_quaternion(q)
    robot_angle = robot_yaw[2] * 180 / math.pi
    victim_pose = convert_from_map_to_robot(250, 300)
    line = str(array.data[1]) + ' ' + str(array.data[3]) + ' (' + str(victim_pose[1]) + ' , ' + str(
        victim_pose[0]) + ') (' + str(robot_odom.pose.pose.position.x) + ' , ' + str(
        robot_odom.pose.pose.position.y) + ') (' + str(
        abs(victim_pose[1] - robot_odom.pose.pose.position.x)) + ' , ' + str(
        abs(victim_pose[0] - robot_odom.pose.pose.position.y)) + ') ' + str(robot_angle) + '\n'
    file.write(line)


if __name__ == '__main__':
    rospy.init_node('collect_y_data')
    robot_odom = nav_msgs.msg.Odometry()
    core_map = nav_msgs.msg.OccupancyGrid()
    victim_info = std_msgs.msg.Float64MultiArray()
    namespace = 'robot0'
    y_list = []
    file = open('../data/hot_y_info.aura', 'a+')
    get_core(rospy.wait_for_message('/core/map', nav_msgs.msg.OccupancyGrid))
    rospy.Subscriber('/' + namespace + '/odom', nav_msgs.msg.Odometry, get_odom)
    rospy.Subscriber('/core/map', nav_msgs.msg.OccupancyGrid, get_core)
    rospy.Subscriber('/' + namespace + '/victims/hot', std_msgs.msg.Float64MultiArray, get_victim)
    rospy.spin()

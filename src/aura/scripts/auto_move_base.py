#!/usr/bin/env python3

import rospy
import actionlib
import nav_msgs.msg
import move_base_msgs.msg
import geometry_msgs.msg
import aura.msg


class AutoMoveBase:
    namespace = 'robot0'
    robot_odometry = None
    map_info = None
    move_base_goal = None
    client = None
    black_list = None
    black_list_publisher = None
    cmd_publisher = None

    def __init__(self, namespace='robot0', node_name='AutoMoveBase', anonymous=True):
        self.namespace = namespace
        rospy.init_node(node_name, anonymous=anonymous)
        self.setup_move_base()
        self.get_robot_odom(rospy.wait_for_message('/' + namespace + '/odom', nav_msgs.msg.Odometry))
        self.get_map(rospy.wait_for_message('/core/map', nav_msgs.msg.OccupancyGrid))
        rospy.Subscriber('/' + namespace + '/odom', nav_msgs.msg.Odometry, self.get_robot_odom)
        rospy.Subscriber('/core/map', nav_msgs.msg.OccupancyGrid, self.get_map)
        rospy.Subscriber('/core/black_list', aura.msg.group, self.get_black_list, queue_size=1000)
        self.black_list_publisher = rospy.Publisher('/core/add_to_black_list', aura.msg.data, queue_size=1000)
        self.cmd_publisher = rospy.Publisher('/' + namespace + '/cmd_vel', geometry_msgs.msg.Twist, queue_size=1000)
        self.black_list = []

    def get_robot_odom(self, odometry: nav_msgs.msg.Odometry):
        self.robot_odometry = odometry

    def get_map(self, map: nav_msgs.msg.OccupancyGrid):
        self.map_info = map

    def setup_move_base(self):
        self.client = actionlib.SimpleActionClient('/' + self.namespace + '/move_base',
                                                   move_base_msgs.msg.MoveBaseAction)
        self.client.wait_for_server()
        self.move_base_goal = move_base_msgs.msg.MoveBaseGoal()

    def send_goal(self, goal_x, goal_y):
        self.client.cancel_all_goals()
        goal = geometry_msgs.msg.PoseStamped()
        goal.header.frame_id = "/map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.orientation.w = 1
        self.move_base_goal.target_pose = goal
        self.client.send_goal(self.move_base_goal, self.goal_status)

    # goal status--- PENDING=0--- ACTIVE=1---PREEMPTED=2--SUCCEEDED=3--ABORTED=4---REJECTED=5--PREEMPTING=6---RECALLING=7---RECALLED=8---LOST=9
    def goal_status(self, data1, data2):
        pass

    def get_black_list(self, new_black_list: aura.msg):
        self.black_list = new_black_list

    def convert_from_robot_to_map(self, robot_y, robot_x) -> tuple:
        map_x = (robot_x - self.map_info.info.origin.position.x) // self.map_info.info.resolution
        map_y = (robot_y - self.map_info.info.origin.position.y) // self.map_info.info.resolution
        return map_y, map_x

    def convert_from_map_to_robot(self, map_y, map_x) -> tuple:
        robot_x = (map_x * self.map_info.info.resolution) + self.map_info.info.origin.position.x
        robot_y = (map_y * self.map_info.info.resolution) + self.map_info.info.origin.position.y
        return robot_y, robot_x

#!/usr/bin/env python3

import rospy
import actionlib
import nav_msgs.msg
import move_base_msgs.msg
import geometry_msgs.msg
import aura.msg


class AutoMoveBase:
    namespace = 'robot0'
    goal_publisher = None
    robot_odometry = None
    map_info = None
    move_base_goal = None
    client = None
    cmd_publisher = None

    def __init__(self, namespace='robot0', node_name='AutoMoveBase', anonymous=True):
        self.namespace = namespace
        rospy.init_node(node_name, anonymous=anonymous)
        self.setup_move_base()
        self.get_robot_odom(rospy.wait_for_message('/' + namespace + '/odom', nav_msgs.msg.Odometry))
        self.get_map(rospy.wait_for_message('/core/map', nav_msgs.msg.OccupancyGrid))
        rospy.Subscriber('/' + namespace + '/odom', nav_msgs.msg.Odometry, self.get_robot_odom)
        rospy.Subscriber('/core/map', nav_msgs.msg.OccupancyGrid, self.get_map)
        self.cmd_publisher = rospy.Publisher('/' + namespace + '/cmd_vel', geometry_msgs.msg.Twist, queue_size=1000)
        self.goal_publisher = rospy.Publisher('/' + namespace + '/goal_pose', aura.msg.goal, queue_size=1000)
        rospy.Subscriber('/core/goal_publisher', aura.msg.multi_goal, self.all_goals, queue_size=1000)

    def get_robot_odom(self, odometry):
        self.robot_odometry = odometry

    def get_map(self, map):
        self.map_info = map

    def setup_move_base(self):
        self.client = actionlib.SimpleActionClient('/' + self.namespace + '/move_base',
                                                   move_base_msgs.msg.MoveBaseAction)
        self.client.wait_for_server()
        self.move_base_goal = move_base_msgs.msg.MoveBaseGoal()

    def send_goal(self, goal_x, goal_y):
        # self.client.cancel_all_goals()
        goal_pose = aura.msg.goal()
        goal_pose.source = self.namespace
        goal_pose.pose.data_float = [goal_x, goal_y]
        self.goal_publisher.publish(goal_pose)
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

    def all_goals(self, goals):
        pass

    def convert_from_robot_to_map(self, robot_y, robot_x):
        map_x = round((robot_x - self.map_info.info.origin.position.x) / self.map_info.info.resolution)
        map_y = round((robot_y - self.map_info.info.origin.position.y) / self.map_info.info.resolution)
        return map_y, map_x

    def convert_from_map_to_robot(self, map_y, map_x):
        robot_x = (map_x * self.map_info.info.resolution) + self.map_info.info.origin.position.x
        robot_y = (map_y * self.map_info.info.resolution) + self.map_info.info.origin.position.y
        return robot_y, robot_x

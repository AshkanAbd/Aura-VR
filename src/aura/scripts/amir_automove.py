import rospy
import actionlib_msgs
import actionlib
import nav_msgs.msg
import std_msgs.msg
import move_base_msgs.msg

name = 'robo1'
map_info = nav_msgs.msg.OccupancyGrid.info
corent_goal_x = 10000
corent_goal_y = 10000


def get_robot_odom(odometry: nav_msgs.msg.Odometry):
    global robot_x, robot_y, robot_odom
    robot_x = odometry.pose.pose.position.x
    robot_y = odometry.pose.pose.position.y
    robot_odom = odometry


def convert_from_robot_to_local_map(robot_y: int, robot_x: int):
    global local_map
    map_x = (robot_x - local_map.info.origin.position.x) // local_map.info.resolution
    map_y = (robot_y - local_map.info.origin.position.y) // local_map.info.resolution
    return map_y, map_x


def convert_from_robot_to_map(robot_y: int, robot_x: int):
    global map_info
    map_x = (robot_x - map_info.info.origin.position.x) // map_info.info.resolution
    map_y = (robot_y - map_info.info.origin.position.y) // map_info.info.resolution
    return map_y, map_x


def convert_from_map_to_robot(map_y: int, map_x: int):
    global map_info
    robot_x = ((map_x) * map_info.info.resolution) + map_info.info.origin.position.x
    robot_y = ((map_y) * map_info.info.resolution) + map_info.info.origin.position.y
    return robot_y, robot_x


def get_map(map: nav_msgs.msg.OccupancyGrid):
    global map_x, map_y
    map_x = int(map.info.width - (100 - (4 * (abs(map.info.origin.position.x) / 10))))
    map_y = int(map.info.height - (100 - (4 * (abs(map.info.origin.position.y) / 10))))


# def generate_goal():
#     global corent_goal_x, corent_goal_y, robot_x, robot_y, goal_x, goal_y


def move_base_clinet(goal_x, goal_y):
    client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
    client.wait_for_server()
    goal = move_base_msgs.msg.MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position_x = goal_x
    goal.target_pose.pose.position_y = goal_y
    client.send_goal(goal)
    rospy.sleep(1)


if __name__ == '__main__':
    name = 'robo1'
    rospy.init_node('core')
    rospy.Subscriber('/core', nav_msgs.msg.OccupancyGrid)
    rospy.spin()

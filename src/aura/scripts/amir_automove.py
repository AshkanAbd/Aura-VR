import rospy
import actionlib
import nav_msgs.msg
import move_base_msgs.msg
import geometry_msgs.msg
import numpy as np

map = nav_msgs.msg.OccupancyGrid
odometry = nav_msgs.msg.Odometry
map_info = nav_msgs.msg.OccupancyGrid.info
goal = geometry_msgs.msg.PoseStamped()
corent_goal_x = 10000
corent_goal_y = 10000


def get_robot_odom(odometry):
    global robot_x, robot_y, robot_odom
    robot_x = odometry.pose.pose.position.x
    robot_y = odometry.pose.pose.position.y
    robot_odom = odometry


def convert_from_robot_to_local_map(robot_y, robot_x):
    global local_map
    map_x = (robot_x - local_map.info.origin.position.x) // local_map.info.resolution
    map_y = (robot_y - local_map.info.origin.position.y) // local_map.info.resolution
    return map_y, map_x


def convert_from_robot_to_map(robot_y, robot_x):
    global map_info
    map_x = (robot_x - map_info.info.origin.position.x) // map_info.info.resolution
    map_y = (robot_y - map_info.info.origin.position.y) // map_info.info.resolution
    return map_y, map_x


def convert_from_map_to_robot(map_y, map_x):
    global map_info
    robot_x = ((map_x) * map_info.info.resolution) + map_info.info.origin.position.x
    robot_y = ((map_y) * map_info.info.resolution) + map_info.info.origin.position.y
    return robot_y, robot_x


def get_map(map: nav_msgs.msg.OccupancyGrid):
    global map_x, map_y , map_info
    map_info = map
    map_x = map.info.width
    map_y = map.info.height
    map_info = np.asarray(map.data)
    print(map_info)
    map_info = map_info.reshape(map_y, map_x)
    b = []
    for i in range(map_y//62):
        for j in range(map_x//62):
            b.append(map_info[i * 16:i * 16+ 16, j * 16:j * 16 + 16])
    print(b)


# def generate_goal():
#     global corent_goal_x, corent_goal_y, robot_x, robot_y, goal_x, goal_y


def setup_move_base():
    global client, move_base_goal
    client = actionlib.SimpleActionClient('/' + name + '/move_base', move_base_msgs.msg.MoveBaseAction)
    client.wait_for_server()
    move_base_goal = move_base_msgs.msg.MoveBaseGoal()


def move_base_clinet(goal_x, goal_y):
    global client, move_base_goal
    goal = geometry_msgs.msg.PoseStamped()
    goal.header.frame_id = "/map"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = goal_x
    goal.pose.position.y = goal_y
    goal.pose.orientation.w = 1
    move_base_goal.target_pose = goal
    client.send_goal(move_base_goal)


client = None
move_base_goal = None

if __name__ == '__main__':
    name = 'robot0'
    rospy.init_node('core1')
    setup_move_base()
    print('hey')
    rospy.Subscriber('/core', nav_msgs.msg.OccupancyGrid, get_map)
    rospy.spin()

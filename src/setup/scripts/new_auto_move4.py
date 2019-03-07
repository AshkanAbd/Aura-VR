#!/usr/bin/env python3.5

import math
import create_map
import rospy
import random
import numpy as np
import nav_msgs.msg
import std_msgs.msg
import actionlib.msg
import geometry_msgs.msg
import actionlib
import move_base_msgs.msg
import trajectory_msgs.msg
import actionlib_msgs.msg
import costmap_2d.msg

namespace = "aura4"
cluster = "4"
core_map = nav_msgs.msg.OccupancyGrid.data
crop_map = nav_msgs.msg.OccupancyGrid.data
local_map = nav_msgs.msg.OccupancyGrid.data
global_map = nav_msgs.msg.OccupancyGrid.data
map_info = nav_msgs.msg.OccupancyGrid.info
robot_odometry = nav_msgs.msg.Odometry
cluster_map = np.asarray([])
designed_local_costmap = np.asarray([])
designed_global_costmap = np.asarray([])
goal_status = actionlib_msgs.msg.GoalStatusArray
client = None
creator = create_map.creator
move_client = None
current_goal_x = -1000
current_goal_y = -1000
map_size_x = 0
map_size_y = 0
activate = [False, False, False, False]
in_start = False
robot_x = 1000
robot_y = 1000
corners = [[]]
end = False
del_additional = False


def get_map(map: nav_msgs.msg.OccupancyGrid):
    global core_map, map_info, map_size_x, map_size_y, crop_map, robot_odometry, current_goal_x, current_goal_y, log
    if del_additional:
        map = removing_additional(map)
    core_map = map.data
    map_info = map
    crop_map = get_crop_map(map)
    map_size_x = int(map.info.width - (100 - (4 * (abs(map.info.origin.position.x) / 10))))
    map_size_y = int(map.info.height - (100 - (4 * (abs(map.info.origin.position.y) / 10))))
    core_map = np.asarray(map.data).reshape(map.info.height, map.info.width)
    robot_map_y, robot_map_x = convert_from_robot_to_map(robot_odometry.pose.pose.position.y,
                                                         robot_odometry.pose.pose.position.x)
    goal_map_y, goal_map_x = convert_from_robot_to_map(current_goal_y, current_goal_x)
    destination = math.sqrt(math.pow(robot_map_y - goal_map_y, 2) + math.pow(robot_map_x - goal_map_x, 2))
    activate[0] = True
    if crop_map[int(goal_map_y - 100), int(goal_map_x - 100)] == 100:
        print("WALL!!!")
        generate_goal()
    if destination <= 3:
        generate_goal()


def get_local_costmap(local_costmap: nav_msgs.msg.OccupancyGrid):
    global current_goal_y, current_goal_x, local_map, designed_local_costmap, robot_x, robot_y
    local_map = local_costmap
    activate[3] = True
    if current_goal_x != -1000:
        local_goal_y, local_goal_x = convert_from_robot_to_local_map(current_goal_y, current_goal_x)
        local_robot_y, local_robot_x = convert_from_robot_to_local_map(robot_y, robot_x)
        designed_local_costmap = np.asarray(local_costmap.data, int).reshape(local_costmap.info.height,
                                                                             local_costmap.info.width)
        # local_x = (local_goal_x - local_robot_x)
        # local_y = (local_goal_y - local_robot_y)
        # print(str(local_x) + "   " + str(local_y))
        if 0 <= local_goal_x < 100 and 0 <= local_goal_y < 100:
            if designed_local_costmap[int(local_goal_y), int(local_goal_x)] >= 30:
                generate_goal()


def get_cluster(cluster_map_cb: std_msgs.msg.Float64MultiArray):
    global cluster_map
    cluster_map = np.asarray(cluster_map_cb.data).reshape(get_shape())
    activate[2] = True


def removing_additional(map: nav_msgs.msg.OccupancyGrid):
    global creator
    new_map = np.asarray(map.data).reshape(map.info.height, map.info.width)
    # creator.get_matrix(new_map.copy)
    # new_map = creator.remove_addition()
    new_map = remove_addition(new_map.copy())
    new_map = new_map.reshape(1, map.info.height * map.info.width)
    map.data = new_map[0].tolist()
    return map




def get_goal_status(status: actionlib_msgs.msg.GoalStatusArray):
    global creator, del_additional, core_map, goal_status
    goal_status = status
    current_status = status.status_list[0].status
    if current_status == 3:
        generate_goal()
    elif current_status == 4:
        # creator.get_matrix(core_map)
        # creator.remove_addition()
        remove_addition(core_map)
        del_additional = True
        generate_goal()
        print("removing additional")
    elif current_status == 5:
        # creator.get_matrix(core_map)
        # creator.remove_addition()
        remove_addition(core_map)
        del_additional = True
        generate_goal()
        print("removing additional")
    elif current_status == 9:
        # creator.get_matrix(core_map)
        # creator.remove_addition()
        remove_addition(core_map)
        del_additional = True
        generate_goal()
        print("removing additional")



def get_crop_map(map: nav_msgs.msg.OccupancyGrid):
    global core_map
    crop_map = np.asarray(core_map)
    map_size_x = int(map.info.width - (100 - (4 * (abs(map.info.origin.position.x) / 10))))
    map_size_y = int(map.info.height - (100 - (4 * (abs(map.info.origin.position.y) / 10))))
    crop_map = crop_map.reshape(map.info.height, map.info.width)
    crop_map = crop_map[100:map_size_y, 100:map_size_x]
    return crop_map


def get_shape():
    global map_info, cluster
    map_size_x = int(map_info.info.width - (100 - (4 * (abs(map_info.info.origin.position.x) / 10))))
    map_size_y = int(map_info.info.height - (100 - (4 * (abs(map_info.info.origin.position.y) / 10))))
    mid_x = (100 + map_size_x) / 2
    mid_y = (100 + map_size_y) / 2
    if cluster == "1":
        return math.ceil(mid_y) - 100, math.floor(mid_x) - 100
    elif cluster == "2":
        return math.floor(mid_y) - 100, math.floor(mid_x) - 100
    elif cluster == "3":
        return math.floor(mid_y) - 100, math.ceil(mid_x) - 100
    else:
        return math.ceil(mid_y) - 100, math.ceil(mid_x) - 100


def get_odometry(odometry: nav_msgs.msg.Odometry):
    global robot_odometry, robot_x, robot_y
    robot_odometry = odometry
    robot_x = odometry.pose.pose.position.x
    robot_y = odometry.pose.pose.position.y
    activate[1] = True


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


def convert_from_cluster_to_map(cluster_y: int, cluster_x: int):
    # global map_info, cluster
    map_size_x = int(map_info.info.width - (100 - (4 * (abs(map_info.info.origin.position.x) / 10))))
    map_size_y = int(map_info.info.height - (100 - (4 * (abs(map_info.info.origin.position.y) / 10))))
    mid_x = (100 + map_size_x) / 2
    mid_y = (100 + map_size_y) / 2
    if cluster == "1":
        return math.floor(mid_y) + cluster_y, cluster_x + 100
    elif cluster == "2":
        return cluster_y + 100, cluster_x + 100
    elif cluster == "3":
        return cluster_y + 100, cluster_x + math.floor(mid_x)
    else:
        return cluster_y + math.floor(mid_y), cluster_x + math.floor(mid_x)


def generate_goal():
    global in_start, robot_odometry, corners, end, map_size_x, map_size_y, core_map
    print("generating goal")
    map_size_x = int(map_info.info.width - (100 - (4 * (abs(map_info.info.origin.position.x) / 10))))
    map_size_y = int(map_info.info.height - (100 - (4 * (abs(map_info.info.origin.position.y) / 10))))
    reshape_map = np.asarray(map_info.data).reshape(map_info.info.height, map_info.info.width)
    robot_map_y, robot_map_x = convert_from_robot_to_map(robot_odometry.pose.pose.position.y,
                                                         robot_odometry.pose.pose.position.x)
    reshape_map = reshape_map[int(robot_map_y - 30):int(robot_map_y + 30), int(robot_map_x - 30):int(robot_map_x + 30)]
    corners = [[0, 0], [0, 38], [38, 0], [38, 38]]
    x = []
    y = []
    z = []
    for corner in corners:
        if reshape_map[corner[0], corner[1]] == -1:
            x.append([corner[0], corner[1]])
    if len(x) != 0:
        a = random.randint(0, len(x) - 1)
        goal_y, goal_x = convert_from_map_to_robot(robot_map_y - 20 + x[a][0], robot_map_x - 20 + x[a][1])
        move_base_client(goal_y, goal_x)
    else:
        d = np.where(reshape_map == -1)
        if len(d[0]) != 0:
            d = np.append(d[0], d[1]).reshape(2, len(d[0]))
            for j, i in np.ndindex(d.shape):
                if j == 0:
                    continue
                y.append(math.sqrt(((d[0, i] - 20) ** 2) + ((d[1, i] - 20) ** 2)))
            index = y.index(max(y))
            goal_y, goal_x = convert_from_map_to_robot(robot_map_y - 20 + d[0, index], robot_map_x - 20 + d[1, index])
            move_base_client(goal_y, goal_x)
        else:
            all_map = np.asarray(map_info.data).reshape(map_info.info.height, map_info.info.width)
            e = np.where(all_map == -1)
            e = np.append(e[0], e[1]).reshape(2, len(e[0]))
            for j, i in np.ndindex(e.shape):
                if j == 0:
                    continue
                z.append([math.sqrt(((e[0, i] - robot_map_y) ** 2) + ((e[1, i] - robot_map_x) ** 2))])
            index = z.index(min(z))
            goal_y, goal_x = convert_from_map_to_robot(robot_map_y + e[0, index],
                                                       robot_map_x + e[0, index])
            move_base_client(goal_y, goal_x)


def move_base_client(goal_y: int, goal_x: int):
    global current_goal_x, current_goal_y, client, move_client
    current_goal_x = goal_x
    current_goal_y = goal_y
    client.wait_for_server()
    goal_pose = geometry_msgs.msg.PoseStamped()
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    goal_pose.pose.orientation.w = 1.0
    goal_pose.pose.orientation.z = 0
    goal_pose.header.frame_id = "/map"
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    goal_pose.pose.orientation.w = 1.0
    goal_pose.pose.orientation.z = 0
    client.cancel_goals_at_and_before_time(rospy.Time.now())
    move_client.target_pose = goal_pose
    rospy.sleep(0.5)
    # move_client_.send_goal_and_wait(goal=move_client_goal_,execute_timeout = rospy.Duration(300),preempt_timeout = rospy.Duration(1))
    client.send_goal(goal=move_client)
    print(namespace, "sent goal")


def starting():
    global start
    while not activate[0] or not activate[1] or not activate[2] or not activate[3]:
        pass
    generate_goal()


def remove_addition(matrix: np.ndarray):
    matrix_1 = matrix.T
    matrix_2 = operator(matrix.copy())
    matrix_3 = np.array(operator(matrix_1.copy())).T
    matrix_2[matrix_3 == 100] = matrix_3[matrix_3 == 100]
    return matrix_3


def operator(matrix: np.ndarray):
    b = np.where(matrix == 100)
    b = np.append(b[0], b[1]).reshape(2, len(b[0]))
    c = np.random.randint(100, 101, matrix.shape)
    matrix = matrix[b[0, 0]:b[0, b.shape[1] - 1] + 1, np.min(b[1]):np.max(b[1]) + 1]
    c[b[0, 0]:b[0, b.shape[1] - 1] + 1, np.min(b[1]):np.max(b[1]) + 1] = matrix
    matrix = c
    for y, x in np.ndindex(b.shape):
        if y == 0:
            continue
        if x == 0:
            matrix[b[0, 0], 0:b[1, 0]] = 100
        if x == b.shape[1] - 1:
            matrix[b[0, x], b[1, x] + 1:matrix.shape[1]] = 100
        try:
            if b[0, x] != b[0, x + 1]:
                matrix[b[0, x], b[1, x] + 1:matrix.shape[1]] = 100
                matrix[b[0, x + 1], 0:b[1, x + 1]] = 100
        except:
            pass
    return matrix


if __name__ == "__main__":
    rospy.init_node('core', anonymous=True)
    namespace = "aura4"
    cluster = "4"
    end = False
    del_additional = False
    goal_status = actionlib_msgs.msg.GoalStatusArray
    creator = create_map.creator
    core_map = nav_msgs.msg.OccupancyGrid
    map_info = nav_msgs.msg.OccupancyGrid
    local_map = nav_msgs.msg.OccupancyGrid
    global_map = nav_msgs.msg.OccupancyGrid
    robot_odometry = nav_msgs.msg.Odometry
    client = actionlib.ActionClient("/" + namespace + "/move_base", move_base_msgs.msg.MoveBaseAction)
    move_client = move_base_msgs.msg.MoveBaseGoal()
    cluster_map = np.asarray([])
    rospy.Subscriber("/core/map", nav_msgs.msg.OccupancyGrid, get_map)
    rospy.Subscriber("/" + namespace + "/odom", nav_msgs.msg.Odometry, get_odometry)
    rospy.Subscriber("/" + namespace + "/move_base/local_costmap/costmap", nav_msgs.msg.OccupancyGrid,
                     get_local_costmap)
    rospy.Subscriber("/" + namespace + "/move_base/status", actionlib_msgs.msg.GoalStatusArray, get_goal_status)
    rospy.Subscriber("/core/cluster" + str(cluster), std_msgs.msg.Float64MultiArray, get_cluster)
    starting()
    in_start = False
    rospy.spin()

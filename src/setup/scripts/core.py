#!/usr/bin/env python3.5

import rospy
import nav_msgs.msg
import numpy as np
import rospy.msg
import std_msgs.msg
import math

robot1_ns = "aura1"
robot2_ns = "aura2"
robot3_ns = "aura3"
robot4_ns = "aura4"
robot5_ns = "aura4"
robot6_ns = "aura6"
core_map = nav_msgs.msg.OccupancyGrid()
core_header = nav_msgs.msg.OccupancyGrid()
core_info = nav_msgs.msg.OccupancyGrid()
cluster1 = np.asarray([[0]])
cluster2 = np.asarray([[0]])
cluster3 = np.asarray([[0]])
cluster4 = np.asarray([[0]])
activate = False


def get_maps(data: nav_msgs.msg.OccupancyGrid, robot: str):
    global core_map, core_header, core_info, activate
    width = data.info.width
    height = data.info.height
    try:
        if core_map is None:
            core_map = np.random.randint(-1, 0, (1, width * height))
            new_map = np.asarray(data.data).reshape(1, width * height)
            # print(robot)
            merging(core_map, new_map, data)
        else:
            new_map = np.asarray(data.data).reshape(1, width * height)
            # print(robot)
            merging(core_map, new_map, data)
    finally:
        core_header = data.header
        core_info = data.info
        activate = True
        # core_info.origin.position.x = 0.025 * data.info.origin.position.x + 0.5
        # core_info.origin.position.y = 0.025 * data.info.origin.position.y + 0.5


def merging(core_map: np.ndarray, new_map: np.ndarray, data: nav_msgs.msg.OccupancyGrid):
    map_size_x = int(data.info.width - (100 - (4 * (abs(data.info.origin.position.x) / 10))))
    map_size_y = int(data.info.height - (100 - (4 * (abs(data.info.origin.position.y) / 10))))
    sub_map = np.random.randint(-1, 0, (map_size_y - 94, map_size_x - 94))
    check_map = new_map.copy().reshape(data.info.height, data.info.width)
    check_map[97:map_size_y + 3, 97:map_size_x + 3] = sub_map
    if (not check_map.__contains__(0)) or (not check_map.__contains__(100)):
        core_map[np.where(core_map == -1)] = new_map[np.where(core_map == -1)]
        core_map[np.where(new_map == 100)] = new_map[np.where(new_map == 100)]
        clustering(core_map.copy(), data)


def clustering(map: np.ndarray, map_info: nav_msgs.msg.OccupancyGrid):
    global cluster1, cluster2, cluster3, cluster4
    map_size_x = int(map_info.info.width - (100 - (4 * (abs(map_info.info.origin.position.x) / 10))))
    map_size_y = int(map_info.info.height - (100 - (4 * (abs(map_info.info.origin.position.y) / 10))))
    map = map.reshape(map_info.info.height, map_info.info.width)
    mid_x = (100 + map_size_x) / 2
    mid_y = (100 + map_size_y) / 2
    cluster1 = map[math.ceil(mid_y):map_size_y, 100:math.floor(mid_x)]
    cluster2 = map[100:math.floor(mid_y), 100:math.floor(mid_x)]
    cluster3 = map[100:math.floor(mid_y), math.ceil(mid_x):map_size_x]
    cluster4 = map[math.ceil(mid_y):map_size_y, math.ceil(mid_x):map_size_x]


def publisher():
    global cluster1, cluster2, cluster3, cluster4
    cluster1_publisher = rospy.Publisher("/core/cluster1", std_msgs.msg.Float64MultiArray, queue_size=10)
    cluster2_publisher = rospy.Publisher("/core/cluster2", std_msgs.msg.Float64MultiArray, queue_size=10)
    cluster3_publisher = rospy.Publisher("/core/cluster3", std_msgs.msg.Float64MultiArray, queue_size=10)
    cluster4_publisher = rospy.Publisher("/core/cluster4", std_msgs.msg.Float64MultiArray, queue_size=10)
    map_publisher = rospy.Publisher("/core/map", nav_msgs.msg.OccupancyGrid, queue_size=10)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        new_data = nav_msgs.msg.OccupancyGrid()
        new_data.info = core_info
        new_data.header = core_header
        new_data.data = core_map[0].tolist()
        map_publisher.publish(new_data)
        cluster1_data = std_msgs.msg.Float64MultiArray()
        cluster2_data = std_msgs.msg.Float64MultiArray()
        cluster3_data = std_msgs.msg.Float64MultiArray()
        cluster4_data = std_msgs.msg.Float64MultiArray()
        cluster1_data.data = cluster1.reshape(1, cluster1.shape[0] * cluster1.shape[1]).tolist()[0]
        cluster2_data.data = cluster2.reshape(1, cluster2.shape[0] * cluster2.shape[1]).tolist()[0]
        cluster3_data.data = cluster3.reshape(1, cluster3.shape[0] * cluster3.shape[1]).tolist()[0]
        cluster4_data.data = cluster4.reshape(1, cluster4.shape[0] * cluster4.shape[1]).tolist()[0]
        cluster1_publisher.publish(cluster1_data)
        cluster2_publisher.publish(cluster2_data)
        cluster3_publisher.publish(cluster3_data)
        cluster4_publisher.publish(cluster4_data)
        rate.sleep()


# def send_map():
#     rate = rospy.Rate(5)
#     while not rospy.is_shutdown():
#         rate.sleep()


def listen_map():
    global robot1_ns, robot2_ns, robot3_ns, robot4_ns, robot5_ns, robot6_ns, core_map
    rospy.Subscriber("/" + robot1_ns + "/map", nav_msgs.msg.OccupancyGrid, get_maps, callback_args="aura1")
    rospy.Subscriber("/" + robot2_ns + "/map", nav_msgs.msg.OccupancyGrid, get_maps, callback_args="aura2")
    rospy.Subscriber("/" + robot3_ns + "/map", nav_msgs.msg.OccupancyGrid, get_maps, callback_args="aura3")
    rospy.Subscriber("/" + robot4_ns + "/map", nav_msgs.msg.OccupancyGrid, get_maps, callback_args="aura4")
    rospy.Subscriber("/" + robot5_ns + "/map", nav_msgs.msg.OccupancyGrid, get_maps, callback_args="aura5")
    rospy.Subscriber("/" + robot6_ns + "/map", nav_msgs.msg.OccupancyGrid, get_maps, callback_args="aura6")
    # rospy.spin()


def start():
    while not activate:
        pass
    # send_map()
    publisher()


if __name__ == "__main__":
    rospy.init_node('core', anonymous=True)
    core_map = None
    listen_map()
    start()

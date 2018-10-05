import rospy
import numpy as np
import nav_msgs.msg
import std_msgs.msg
import aura.msg

map = nav_msgs.msg.OccupancyGrid
map_info = nav_msgs.msg.OccupancyGrid.info


def devide():
    global map_info, map_x, map_y
    b = []
    group = aura.msg.group()
    for i in range(map_y // 62):
        for j in range(map_x // 62):
            data = aura.msg.data()
            temp = np.asarray(map_info[i * 16:i * 16 + 16, j * 16:j * 16 + 16], np.int32)
            temp = temp.reshape(256).tolist()
            data.data = temp
            group.array.append(data)
    return group


def get_map(map: nav_msgs.msg.OccupancyGrid):
    global map_x, map_y, map_info, cluster_publisher
    map_info = map
    map_x = map.info.width
    map_y = map.info.height
    map_info = np.asarray(map.data)
    map_info = map_info.reshape(map_y, map_x)
    result = devide()
    cluster_publisher.publish(result)


if __name__ == '__main__':
    map_info = None
    map_x = None
    map_y = None
    rospy.init_node('cluster')
    cluster_publisher = rospy.Publisher('/cluster', aura.msg.group, queue_size=20)
    rospy.Subscriber('/core', nav_msgs.msg.OccupancyGrid, get_map)
    rospy.spin()

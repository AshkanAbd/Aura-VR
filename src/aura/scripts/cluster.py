import rospy
import numpy as np
import nav_msgs.msg

map = nav_msgs.msg.OccupancyGrid
map_info = nav_msgs.msg.OccupancyGrid.info


def dvide(map: nav_msgs.msg.OccupancyGrid):
    global map_info, b, map_x, map_y
    map_info = map
    map_x = map.info.width
    map_y = map.info.height
    map_info = np.asarray(map.data)
    map_info = map_info.reshape(map_y, map_x)
    b = []
    for i in range(map_y // 62):
        for j in range(map_x // 62):
            b = b.append(map_info[i * 16:i * 16 + 16, j * 16:j * 16 + 16])
    print(b)


def pub():
    global cluster_publisher, pup_method, b
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pup_method = dvide(b)
        cluster_publisher.publish(b)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('core1')
    rospy.Subscriber('/core', nav_msgs.msg.OccupancyGrid)
    cluster_publisher = rospy.Publisher('/cluster', nav_msgs.msg.OccupancyGrid, queue_size=20)
    pub()
    rospy.spin()

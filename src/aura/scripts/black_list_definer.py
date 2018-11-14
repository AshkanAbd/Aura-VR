#!/usr/bin/env python3

import rospy
import aura.msg


class BlackListDefiner:
    black_list = None
    black_list_publisher = None
    rate = None

    def __init__(self) -> None:
        rospy.init_node('black_list_definer')
        self.black_list_publisher = rospy.Publisher('/core/black_list', aura.msg.group_float, queue_size=1000)
        rospy.Subscriber('/core/add_to_black_list', aura.msg.data_float, self.get_black_list, queue_size=1000)
        self.black_list = []
        self.rate = rospy.Rate(10)
        self.publish_black_list()

    def get_black_list(self, point: aura.msg.data_float) -> None:
        if point not in self.black_list:
            self.black_list.append(point)

    def publish_black_list(self) -> None:
        while not rospy.is_shutdown():
            msg = aura.msg.group_float()
            msg.array = self.black_list
            self.black_list_publisher.publish(msg)
            self.rate.sleep()


if __name__ == '__main__':
    definer = BlackListDefiner()
    rospy.spin()

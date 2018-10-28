import numpy as np
import random
import block
import auto_move_base
import rospy
import aura.msg


class DFSAutoMove(auto_move_base.AutoMoveBase):
    block_array = []
    robot_block = None
    goal_x = -10000
    goal_y = -10000

    def __init__(self, namespace='robot0', node_name='AutoMoveBase', anonymous=True):
        super().__init__(namespace, node_name, anonymous)
        self.get_blocks(rospy.wait_for_message('/core/blocks', aura.msg.group))
        rospy.Subscriber('/core/blocks', aura.msg.group, self.get_blocks)

    def get_blocks(self, blocks_array: aura.msg.group):
        temp_array = []
        for i in range(0, 256):
            block_obj = block.Block(i, blocks_array.array[i].data, self.map_info.info.height,
                                    self.map_info.info.width)
            temp_array.append(block_obj)
        self.block_array = temp_array
        self.robot_block = self.find_robot_block()

    def find_robot_block(self):
        robot_pose_y = self.robot_odometry.pose.pose.position.y
        robot_pose_x = self.robot_odometry.pose.pose.position.x
        robot_pose = self.convert_from_robot_to_map(robot_pose_y, robot_pose_x)
        y = robot_pose[0] // (self.map_info.info.height // 16)
        x = robot_pose[1] // (self.map_info.info.width // 16)
        robot_block_index = int((y * 16) + x)
        return robot_block_index

    def generating_goal(self, block_index):
        n_shown = np.where(self.block_array[block_index].get_reshaped_block() == -1)
        if len(n_shown[0]) == 0: return False
        rand = random.randint(0, len(n_shown[0]))
        map_goal_x = (self.block_array[block_index].block_width * self.block_array[block_index].column) + n_shown[0][rand]
        map_goal_y = (self.block_array[block_index].block_height* self.block_array[block_index].row) + n_shown[1][rand]
        goal_y, goal_x = self.convert_from_map_to_robot(map_goal_y, map_goal_x)
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.send_goal(goal_x, goal_y)
        print("GOAL PUBLISHED " + str(goal_x) + " , " + str(goal_y))
        return True

    def goal_status(self, data1, data2):
        print(data1)
        self.start(self.robot_block)

    def start(self, block_index):
        if self.generating_goal(block_index):
            return
        neighbors = [self.block_array[block_index].go_up(), self.block_array[block_index].go_down(),
                     self.block_array[block_index].go_left(), self.block_array[block_index].go_right()]
        random.shuffle(neighbors)
        for i in neighbors:
            if self.block_array[i].has_unkown():
                self.generating_goal(i)
                return
        self.start(neighbors[0])

    def current_goal(self):
        return self.goal_x, self.goal_y

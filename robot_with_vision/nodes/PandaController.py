# !/usr/bin/env python

import time
import rospy

# import tf
# from rospy import ServiceProxy, ServiceException
#
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
#
# from ros_utilities import *


class PandaController:

    def __init__(self):
        self.joint_update_delay = 10  # sleep time in ms between sending new joint position

        self.current_joints = [0, 0, 0, 0, 0, 0, 0]

        rospy.init_node("panda_control")
        rospy.Publisher("/joint_states", JointState)
        rospy.Subscriber('/panda_waypoints', GetPlan, self.request_trajectory)

        rospy.loginfo("PandaController server node ready.")

    def joints_update(self, joints_msg):
        # compose joint_state message for each joint and publish them
        return

    def go_to(self, trajectory):
        # iterate through each trajectory and set each series joint positions
        for i in range(len(trajectory)):
            self.joints_update(trajectory[i])
            rospy.sleep(self.joint_update_delay)

    def request_trajectory(self, midpoint, goal):  # probably will be easier to use some ros data type
        start = self.current_joints
        # send message to trajectory node
        # wait for response
        # start go_to function
        return


if __name__ == '__main__':
    panda = PandaController()


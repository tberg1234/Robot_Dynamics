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

        current_joints = [0, 0, 0, 0, 0, 0, 0]

        rospy.init_node("panda_control", anonymous=True)
        rospy.Publisher("/joint_states", JointState, self.joints_update)
        rospy.Subscriber('/panda_waypoints', GetPlan, self.request_trajectory)

        self.ros_rate = rospy.Rate(self.rate)
        rospy.loginfo("PandaController server node ready.")

    def joints_update(self, joints_msg):
        return



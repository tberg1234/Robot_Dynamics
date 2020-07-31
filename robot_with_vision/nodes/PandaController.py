# !/usr/bin/env python

# import time
import rospy

# import tf
# from rospy import ServiceProxy, ServiceException
#
# from std_msgs.msg import String
from sensor_msgs.msg import JointState
# from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
#
# from ros_utilities import *


class PandaController:

    def __init__(self):
        self.joint_update_delay = 10  # sleep time in ms between sending new joint position

        self.current_joints = [0, 0, 0, 0, 0, 0, 0]

        rospy.init_node("panda_control")
        self.joint_state_pub = rospy.Publisher("/joint_states", JointState)
        rospy.Subscriber('/panda_waypoints', GetPlan, self.request_trajectory)

        # start vision node
        # start trajectory node
        rospy.sleep(10)
        rospy.spin()
        rospy.loginfo("PandaController server node ready.")

    def joints_update(self, joints_pos, joints_vel, joints_eff):
        # compose JointState message with position, velocity and effort/force for each joint
        # joints are denoted by name as defined in the xacro
        name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
                'panda_joint5', 'panda_joint6', 'panda_joint7']
        position = []
        velocity = []
        effort = []
        for i in range(len(joints_pos)):
            position.append(joints_pos[i])
            velocity.append(joints_vel[i])
            effort.append(joints_eff[i])
        j_msg = JointState().name = name
        j_msg.position = position
        j_msg.velocity = velocity
        j_msg.effort = effort
        self.joint_state_pub.publish(j_msg)
        return

    def go_to(self, trajectory):
        # takes a trajectory, which is an array of JointState values (pos, vel, effort)
        # iterate through each trajectory point and set each joint variable
        for i in range(len(trajectory)):
            self.joints_update(trajectory[0][i], trajectory[1][i], trajectory[2][i])
            rospy.sleep(self.joint_update_delay)

    def request_trajectory(self, obj_location, midpoint, goal):  # probably will be easier to use some ros data type
        # send message to trajectory node with start, midpoint and end location in XYZ
        # trajectory node should return a
        # wait for response
        # start go_to function
        return


if __name__ == '__main__':
    panda = PandaController()

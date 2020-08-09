# !/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from nav_msgs.srv import GetPlan
from robot_with_vision.msg import centers, points_to_go_to

# PandaController coordinator node: uses custom ROS message to receive a list of block locations and their respective
# colors, then publishes a Path on the /block_points topic, which the Matlab portion uses to calculate required
# kinematics and dynamics, and publishes these results as JointState messages on /joint_states

# Blocks generated by launch file
# PandaController node subscribes to /block_points and waits for Vision node to interpret workspace
# Vision node returns a list of XYZ(?) block locations as a custom message on 'block_points' topic
# PandaController interprets this list, adds a midpoint and destination pose, and then sends this list of
#    desired EE positions to the Trajectory node
# Trajectory node then calculates a long list of joint position/velocity/effort for each joint that together detail each
#    intermediate time step of the robot's movements
# Trajectory node publishes these as JointState messages, which will be picked up by the robot_state_publisher and
#    will animate our virtual model


class PandaController:

    def __init__(self):
        self.joint_update_delay = 10  # sleep time in ms between sending new joint position

        rospy.init_node("panda_control")
        self.joint_state_pub = rospy.Publisher("/joint_states", JointState)
        self.waypoint_pub = rospy.Publisher("/panda_waypoints", GetPlan)
        rospy.Subscriber('/block_points', points_to_go_to, self.request_trajectory)

        rospy.sleep(10)
        rospy.spin()
        rospy.loginfo("PandaController server node ready.")

    @staticmethod
    def request_trajectory(msg):
        # send message to trajectory node with start, midpoint and end location in XYZ
        # prepare '/panda_waypoints' message for trajectory: add midpoint and goal location based on color of block
        waypoints = []
        blue = [.5, .5, .1]
        red = [.5, .6, .1]
        green = [.5, .7, .1]
        for i in range(len(msg.x_points)):
            # block location
            xi = msg.x_points[i]
            yi = msg.y_points[i]
            zi = msg.z_points[i]
            waypoints.append([xi, yi, zi])
            # midpoint; just lifts .2m in z axis
            zm = zi + .2
            waypoints.append([xi, yi, zm])
            # sorted destination location
            if msg.colors[i] == 'blue':
                waypoints.append(blue)
            if msg.colors[i] == 'red':
                waypoints.append(red)
            if msg.colors[i] == 'green':
                waypoints.append(green)

        # prepare Path message
        path_msg = Path()
        for i in range(len(waypoints)):
            pt = PoseStamped()
            pt.pose.position.x = waypoints[i][0]
            pt.pose.position.y = waypoints[i][1]
            pt.pose.position.z = waypoints[i][2]
            path_msg.poses.append(pt)
        return


if __name__ == '__main__':
    panda = PandaController()

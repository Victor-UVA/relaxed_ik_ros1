#! /usr/bin/env python

from math import cos, sin
from enum import Enum, auto
import numpy as np
import rospy
from relaxed_ik_ros1.msg import EEPoseGoals
from geometry_msgs.msg import Pose, PoseArray
from std_msgs import Float32
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseWithCovariance


class States(Enum):
    PLANNING = auto()
    MOVE_TO_HANDLE = auto()
    UNLATCH = auto()
    PULL = auto()
    MOVE_AROUND = auto()
    PUSH = auto()
    ENTER = auto()


class Arm:
    def __init__(self):
        self.state = States.PLANNING
        self.ee_pose_goals_pub = rospy.Publisher(
            '/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=10)

    def send_goal(self, x, y, z, seq, w=1, x_q=0, y_q=0, z_q=0):
        ee_pose = Pose()
        ee_pose_goal = EEPoseGoals()
        ee_pose.position.x = x
        ee_pose.position.y = y
        ee_pose.position.z = z
        # q = tf_conversions.transformations.quaternion_from_euler()
        ee_pose.orientation.w = w
        ee_pose.orientation.x = x_q
        ee_pose.orientation.y = y_q
        ee_pose.orientation.z = z_q

        ee_pose_goal.header.seq = seq
        ee_pose_goal.ee_poses.append(ee_pose)
        self.ee_pose_goals_pub.publish(ee_pose_goal)


def create_arc(handle_pose, handle_to_hinge, num_angles):
    trajectory = []
    angles = np.linspace(0, 90, num_angles)
    for angle in angles:
        trajectory.append([handle_pose[0] + handle_to_hinge*cos(angle),
                           handle_pose[1] + handle_to_hinge*sin(angle), handle_pose[2]])
    return trajectory


def pose_lookup(parent, child):
    tfBuffer = tf2_ros.Buffer()  # may need a rospy.Duration(__) here
    listener = tf2_ros.TransformListener(tfBuffer)
    try:
        trans = tfBuffer.lookup_transform(parent, child, rospy.Time())
        return trans
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Could not find transform for parent: " +
                     str(parent) + "and child: " + str(child))
        rospy.sleep(5)


if __name__ == "__main__":
    rospy.init_node('open_door')
    arm = Arm()
    trajectory = []
    num_angles = 180
    while not rospy.is_shutdown():
        if arm.state == States.PLANNING:
            # Get position of handle relative to camera frame
            handle_pose = rospy.wait_for_message("camera/handle_pose", Pose)
            # Transform to world/map frame and then to arm start pose
            handle_pose = pose_lookup(
                "ur_arm_starting_pose", "camera/handle_pose")  # frame
            # Get dimensions of door
            handle_to_hinge = rospy.wait_for_message(
                "handle_to_hinge", Float32)
            # Create arc
            trajectory = create_arc(handle_pose, handle_to_hinge, num_angles)
            arm.state = States.MOVE_TO_HANDLE
        elif arm.state == States.MOVE_TO_HANDLE:
            # Move EE to right above handle
            arm.send_goal(
                trajectory[0][0], trajectory[0][1], trajectory[0][2]+0.1, seq=1)
            # Move EE to first arc point
            arm.send_goal(*trajectory[0], seq=2)
            arm.state = States.UNLATCH
        elif arm.state == States.UNLATCH:
            # Rotate EE Clockwise
            arm.send_goal(*trajectory[0], seq=3, y_q=1)
            arm.state = States.PULL
        elif arm.state == States.PULL:
            # Follow trajectory to ___
            for i in range(1, num_angles/2):
                arm.send_goal(*trajectory[i], seq=i+3)
            arm.state = States.MOVE_AROUND
        elif arm.state == States.MOVE_AROUND:
            # Unhook EE from handle
            # Reverse base
            # Drive around side
            pass
        elif arm.state == States.PUSH:
            # Follow remaining trajectory
            pass

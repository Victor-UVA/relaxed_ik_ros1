#! /usr/bin/env python3

from audioop import lin2adpcm
import math
from enum import Enum, auto
import numpy as np
import rospy
from relaxed_ik_ros1.msg import EEPoseGoals
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float32
import tf2_ros
import tf_conversions
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ur_interface import end_effector


class States(Enum):
    PLANNING = auto()
    MOVE_TO_HANDLE = auto()
    UNLATCH = auto()
    PULL = auto()
    MOVE_AROUND = auto()
    PUSH = auto()
    ENTER = auto()
    DONE = auto()


class Arm:
    def __init__(self):
        self.state = States.PLANNING
        self.ee_pose_goals_pub = rospy.Publisher(
            '/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=10)
        self.seq = 0
        self.ee = end_effector()

    def send_goal(self, x, y, z, x_q=0, y_q=0, z_q=0):
        ee_pose = Pose()
        ee_pose_goal = EEPoseGoals()
        # if math.sqrt(pow(x, 2)+pow(y, 2)+pow(z, 2)) > 0.85:
        #     rospy.logwarn("Provided goal of (x = "+str(x)+", y = " +
        #                   str(y)+", z = "+str(z)+") exceeds max arm reach.")
        ee_pose.position.x = x
        ee_pose.position.y = y
        ee_pose.position.z = z
        q = tf_conversions.transformations.quaternion_from_euler(
            x_q, y_q, z_q)
        ee_pose.orientation.w = float(q[3])
        ee_pose.orientation.x = float(q[0])
        ee_pose.orientation.y = float(q[1])
        ee_pose.orientation.z = float(q[2])

        rospy.loginfo("Moving arm to position (" +str(x)+", "+str(y)+", "+str(z)+")")# with orientation (w, x, y, z) = ")


        ee_pose_goal.ee_poses.append(ee_pose)
        ee_pose_goal.header.seq = self.seq
        self.ee_pose_goals_pub.publish(ee_pose_goal)
        self.seq += 1


def create_arc(start_pose, radius, resolution, offset =0):
    trajectory = []
    angles = np.linspace(0, 85, resolution)
    for angle in angles:
        trajectory.append([start_pose[0] + offset - radius*math.cos(angle*math.pi/180),
                           start_pose[1] - radius*math.sin(angle*math.pi/180), start_pose[2]])
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


def calculate_collision_angle(door_width, distance_from_door):
    sigma_r = 0.03  # m
    sigma_d = 0.05  # m
    r = door_width + sigma_r
    d = distance_from_door + sigma_d
    theta_m = math.asin(distance_from_door/door_width)
    theta_deriv_d = 1/math.sqrt(pow(r, 2) - pow(d, 2))
    theta_deriv_r = -d/(r*math.sqrt(pow(r, 2) - pow(d, 2)))
    sigma_theta = math.sqrt(
        pow(theta_deriv_d, 2)*pow(sigma_d, 2) + pow(theta_deriv_r, 2)*pow(sigma_r, 2))
    theta = theta_m + sigma_theta
    return theta*180/math.pi # return degrees


def generate_goal(x, y, yaw=0.0, frame="map"):  # yaw in degrees
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    (goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y,
        goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w) = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw*math.pi/180)
    # default: no rotation
    return goal


if __name__ == "__main__":
    rospy.init_node('open_door')
    arm = Arm()
    handle_arc = []
    rate = rospy.Rate(5)
    num_angles = 150
    collision_index = 0
    while not rospy.is_shutdown():
        if arm.state == States.PLANNING:
            rospy.loginfo("Planning")
            # Get position of handle relative to camera frame
            # handle_pose = rospy.wait_for_message("camera/handle_pose", Pose)
            distance_to_base = 0.64

            # Transform to world/map frame and then to arm start pose
            handle_pose = [-0.153, distance_to_base-0.12, 0.13]  # pose_lookup(
            # "ur_arm_starting_pose", "camera/handle_pose")  # frame

            # Get dimensions of door and distance to base
            handle_to_hinge = 0.88  # rospy.wait_for_message(
            # "handle_to_hinge", Float32)
            door_width = 1.01  # rospy.wait_for_message("door_width", Float32)
            door_pose = [handle_pose[0], handle_pose[1] +
                         door_width-handle_to_hinge, handle_pose[2]]  # TODO: Fix this

            # Create handle arc
            handle_arc = create_arc(handle_pose, handle_to_hinge, num_angles, offset=handle_to_hinge)

            # Create door arc
            door_arc = create_arc(door_pose, door_width, num_angles)

            # Calculate collision angle
            collision_angle = calculate_collision_angle(
                door_width, distance_to_base)
            angles = np.linspace(0, 90, num_angles)
            for i in range(len(angles)):
                if angles[i] >= collision_angle:
                    collision_index = i-1  # maybe don't need -1
                    break
            rospy.loginfo("Door will collide at " +
                          str(collision_angle)+" / at index "+str(collision_index))

            # Start move_base stuff
            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            wait = client.wait_for_server()
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            rospy.loginfo("Connected to move base server")

            rospy.loginfo("Done planning")
            arm.state = States.MOVE_TO_HANDLE

        elif arm.state == States.MOVE_TO_HANDLE:
            rospy.loginfo("Grabbing handle")
            # Move EE to right above handle to be able to hook handle
            arm.send_goal(
                handle_arc[0][0], handle_arc[0][1], handle_arc[0][2]+0.1)
            rospy.sleep(10)

            # Move EE to first arc point
            arm.send_goal(*handle_arc[0])
            rospy.sleep(2)

            arm.state = States.UNLATCH

        elif arm.state == States.UNLATCH:
            rospy.loginfo("Unlatching")
            # Rotate EE Clockwise
            arm.send_goal(handle_arc[0][0], handle_arc[0][1], handle_arc[0][2]-0.055, y_q=math.pi/2)
            rospy.sleep(2)

            arm.state = States.PULL #TODO: Reset

        elif arm.state == States.PULL:
            rospy.loginfo("Pulling")
            # Follow trajectory before collision angle
            # angles = np.linspace()
            for i in range(collision_index):
                arm.send_goal(handle_arc[i][0], handle_arc[i][1], handle_arc[i][2]-0.055, y_q=math.pi/2)
                rospy.sleep(0.25)
            rospy.loginfo("Pull completed")
            arm.state = States.MOVE_AROUND #TODO

        elif arm.state == States.MOVE_AROUND:
            rospy.loginfo("Unhooking")
            # Unhook EE from handle
            arm.send_goal(handle_arc[collision_index-1][0], handle_arc[collision_index-1]
                          [1], handle_arc[collision_index-1][2]+0.1)
            rospy.sleep(5)

            # Move to other side of door
            rospy.set_param('/move_base/DWAPlannerROS/', {
                "xy_goal_tolerance": 0.05,
                "yaw_goal_tolerance": 5*math.pi/180,
            })
            # http://wiki.ros.org/base_local_planner
            rospy.set_param('/move_base/TrajectoryPlannerROS/', {
                "xy_goal_tolerance": 0.05,
                "yaw_goal_tolerance": 5*math.pi/180,
            })

            # Disable global costmap
            rospy.set_param("/move_base/NavfnROS/", {
                "cost_factor": 0.0,
                "cost_neutral": 0.0
            })

            rospy.set_param("/move_base/local_costmap/inflation/inflation_radius", 0.001)

            rospy.loginfo("Starting base movement")
            client.send_goal(generate_goal(-0.6, 0.0, -330, frame="map"))
            client.wait_for_result()

            rospy.loginfo("Homing")
            arm.send_goal(0,0,0)
            rospy.sleep(10)

            # Home arm
            # arm.ee.send_to_home()
            # rospy.sleep(5)
            # arm.ee.send_transforms()
            # rospy.sleep(5)
            # arm.ee.send_joint_command() 
            # rospy.sleep(10)


            # Move to other side of door
            rospy.loginfo("Moving into position")
            client.send_goal(generate_goal(0.68, 0.59, -37, frame="map"))
            client.wait_for_result()
            
            arm.state = States.PUSH

        elif arm.state == States.PUSH:
            rospy.loginfo("Pushing")
            arm.send_goal(handle_arc[collision_index][0], handle_arc[collision_index]
                              [1]+0.3, handle_arc[collision_index][2]-0.25)  # TODO: use the z component of ur_base
            rospy.sleep(5)
            # Follow remaining trajectory pushing parallel to ground
            for i in range(collision_index, len(angles)):
                arm.send_goal(handle_arc[i][0], handle_arc[i]
                              [1]+0.3, handle_arc[i][2]-0.3)  # TODO: use the z component of ur_base
                rospy.sleep(0.2)

            arm.state = States.ENTER

        elif arm.state == States.ENTER:
            rospy.loginfo("Homing")
            arm.send_goal(0,0,0)
            rospy.sleep(10)

            rospy.loginfo("Entering room")
            client.send_goal(generate_goal(2.5, -0.2, frame="map"))
            client.wait_for_result()

            rospy.signal_shutdown("Done")
        rospy.sleep(5)
        # rate.sleep()

#!/usr/bin/env python3
from __future__ import print_function
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
# from relaxed_ik_node import JointAngles
from relaxed_ik_ros1.msg import JointAngles
import numpy as np
import tf2_ros
import geometry_msgs.msg
import tf
import tf_conversions


class end_effector():
    def __init__(self):
        self.angular_pose_pub = rospy.Publisher(
            "ur/ur_arm_scaled_pos_joint_traj_controller/command", JointTrajectory, queue_size=10)
        self.joint_states_sub = rospy.Subscriber(
            "joint_states", JointState, self.js_cb)
        self.rleaxed_ik_joint_angles = rospy.Subscriber(
            "relaxed_ik/joint_angle_solutions", JointAngles, self.rik_ja_cb)
        self.joint_states = np.zeros(6)
        self.init_state = np.zeros(6)
        self.joint_command = np.array([-2.617993878,
                                       -0.5235987756,
                                       -1.570796327,
                                       0.0,
                                       1.570796327,
                                       0.0])

        # [-1.2,
        # -0.38,
        # 3.14,
        # -1.57,
        # -1.57,
        # -1.57]

        # [2.36,
        # -2.36,
        # 0,
        # 3.14,
        # -1.57,
        # 0]
    def js_cb(self, data):
        time = rospy.Time.now()
        self.joint_states = np.array(data.position)
        # print(self.joint_states)

    def initialize_state(self):
        self.init_state = self.joint_states
        # print(self.joint_states)
        # print('Initial State is: ',self.init_state)

    def rik_ja_cb(self, data):
        joint_angles = np.array(data.angles.data)
        self.joint_command[0:3] = np.flip(joint_angles)[3:]
        self.joint_command[3:] = joint_angles[3:]
        # print(self.joint_command)

    def send_to_home(self):
        msg = JointTrajectory()
        point = JointTrajectoryPoint()
        point.positions = [-2.617993878,
                           -0.5235987756,
                           -1.570796327,
                           0.0,
                           1.570796327,
                           0.0]
        point.velocities = []
        point.accelerations = []
        point.effort = []
        point.time_from_start.secs = 5
        msg.points.append(point)
        msg.joint_names = ['ur_arm_elbow_joint',
                           'ur_arm_shoulder_lift_joint',
                           'ur_arm_shoulder_pan_joint',
                           'ur_arm_wrist_1_joint',
                           'ur_arm_wrist_2_joint',
                           'ur_arm_wrist_3_joint']
        msg.header.stamp.secs = 0
        # print(msg)

        self.angular_pose_pub.publish(msg)

    def send_to_start(self):
        msg = JointTrajectory()
        point = JointTrajectoryPoint()
        point.positions = [2.36,
                           -2.36,
                           0,
                           3.14,
                           -1.57,
                           0]
        point.velocities = []
        point.accelerations = []
        point.effort = []
        point.time_from_start.secs = 5
        msg.points.append(point)
        msg.joint_names = ['ur_arm_elbow_joint',
                           'ur_arm_shoulder_lift_joint',
                           'ur_arm_shoulder_pan_joint',
                           'ur_arm_wrist_1_joint',
                           'ur_arm_wrist_2_joint',
                           'ur_arm_wrist_3_joint']
        msg.header.stamp.secs = 1
        # print(msg)

        self.angular_pose_pub.publish(msg)

    def send_joint_command(self):
        msg = JointTrajectory()
        point = JointTrajectoryPoint()
        # print(self.joint_command)
        # point.positions = [2.32,
        #                     -2.89,
        #                     3.14,
        #                     -2.55,
        #                     -1.56,
        #                     -3.14]
        point.positions = self.joint_command.tolist()
        point.velocities = []
        point.accelerations = []
        point.effort = []
        #point.time_from_start.secs = 1
        msg.points.append(point)
        msg.joint_names = ['ur_arm_elbow_joint',
                           'ur_arm_shoulder_lift_joint',
                           'ur_arm_shoulder_pan_joint',
                           'ur_arm_wrist_1_joint',
                           'ur_arm_wrist_2_joint',
                           'ur_arm_wrist_3_joint']
        msg.header.stamp.secs = 0

        # Update goal_time parameter
        # print("Joint Command")
        # print(self.joint_command)
        # print("Joint State")
        # print(self.joint_states)
        error = np.abs(self.joint_command - self.joint_states)
        # print('error',error)
        w = np.array([2, 2, 2, 0.2, 0.2, 0.2])
        weight = np.abs(np.sum(np.multiply(w, error)))
        # print('weight',max(weight,0.5))
        point.time_from_start = rospy.Duration(max(weight, 0.5))
        # rospy.set_param('/scaled_vel_joint_traj_controller/constraints/goal_time',5*int(weight))

        self.angular_pose_pub.publish(msg)

    def send_transforms(self):
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        ee_link = self.transform(self.pose_lookup("ur_arm_wrist_2_link", "ur_arm_flange"),
                                 "ur_arm_wrist_2_link", "ur_arm_ee_link", 0, -1.5707, 1.5707)
        starting_pose = self.transform(self.pose_lookup(
            "ur_arm_base_link", "ur_arm_flange"), "ur_arm_base_link", "ur_arm_starting_pose", 0, -1.5707, 3.14159)
        if ee_link is not None and starting_pose is not None:
            broadcaster.sendTransform([ee_link, starting_pose])

    def pose_lookup(self, parent, child):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        rospy.sleep(1)

        try:
            trans = tfBuffer.lookup_transform(parent, child, rospy.Time())
            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(5)

    def transform(self, pose, parent, child, x, y, z):
        if pose is not None:
            static_transformStamped = geometry_msgs.msg.TransformStamped()

            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = parent
            static_transformStamped.child_frame_id = child

            static_transformStamped.transform.translation.x = float(
                pose.transform.translation.x)
            static_transformStamped.transform.translation.y = float(
                pose.transform.translation.y)
            static_transformStamped.transform.translation.z = float(
                pose.transform.translation.z)

            quat = tf.transformations.quaternion_from_euler(
                float(x), float(y), float(z))
            static_transformStamped.transform.rotation.x = quat[0]
            static_transformStamped.transform.rotation.y = quat[1]
            static_transformStamped.transform.rotation.z = quat[2]
            static_transformStamped.transform.rotation.w = quat[3]

            return static_transformStamped
        else:
            return None


if __name__ == '__main__':

    rospy.init_node('command_relaxIK')
    rospy.set_param(
        'ur/ur_arm_scaled_vel_joint_traj_controller/constraints/goal_time', 10)
    # rate1 = rospy.Rate(1)

    ee = end_effector()
    # ee.initialize_state()           # Get initial State
    rospy.sleep(5)                    # Wait for 5 seconds
    ee.send_to_home()               # send to home position
    rospy.sleep(8)                    # Wait for 5 seconds
    rate = rospy.Rate(2)
    ee.send_transforms()
    print("ready to start tasks")
    while not rospy.is_shutdown():
        ee.send_joint_command()     # send commands
        rate.sleep()                # Wait for 2 seconds

# Author: Hudson Burke

import threading
import rospy
from relaxed_ik_ros1.msg import EEPoseGoals
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from relaxed_ik_ros1.msg import JointAngles
from geometry_msgs.msg import Pose, TransformStamped
import transformations as T
from tf_functions import pose_lookup, transform
import numpy as np
from tf2_ros import StaticTransformBroadcaster
import yaml


class Arm:
    # TODO: load from config file
    def __init__(self, config_file_name: str, is_path_to_file=False, home=[-2.617993878, -0.5235987756, -1.570796327, 0.0, 1.570796327, 0.0]):
        self.ee_pose_goals_pub = rospy.Publisher(
            '/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=10)
        self.angular_pose_pub = rospy.Publisher(
            "ur/ur_arm_scaled_pos_joint_traj_controller/command", JointTrajectory, queue_size=10)
        self.joint_states_sub = rospy.Subscriber(
            "ur/joint_states", JointState, self.js_cb)
        self.relaxed_ik_joint_angles = rospy.Subscriber(
            "/relaxed_ik/joint_angle_solutions", JointAngles, self.rik_ja_cb)

        self.joint_states = np.zeros(6)
        self.init_state = np.zeros(6)
        self.joint_command = np.array(home)

        self.seq = 0
        if not is_path_to_file:
            path = '../relaxed_ik_core/config/info_files/' + \
                str(config_file_name)
        else:
            path = config_file_name

        self.home = np.zeros(6)
        try:
            with open(path) as f:
                file = yaml.safe_load(f)
                starting_config = np.array(file['starting_config'])
                self.home[:3] = np.flip(starting_config)[:3]
                self.home[3:] = starting_config[3:]
        except:
            rospy.logerr("Could not open file: " +
                         str(config_file_name)+" or has missing/invalid starting_config.")
            rospy.logerr("Home will be set to all zeros.")

        # UR Interface
        rospy.loginfo("Initializing UR Interface...")
        rospy.sleep(1)
        rospy.loginfo("Homing...")
        self.initial_home()
        # wait until reaches home
        rospy.sleep(10)
        rospy.loginfo("Sending transforms")
        self.send_transforms()
        rospy.sleep(1)
        # clear the relaxed_ik commands
        # self.clear_goal()
        self.send_to_home()
        rospy.sleep(1)
        rospy.loginfo("Ready to begin tasks")

        thread_loop = threading.Thread(target=self.joint_command_loop)
        thread_loop.start()

    def rik_ja_cb(self, data):
        joint_angles = np.array(data.angles.data)
        self.joint_command[0:3] = np.flip(joint_angles)[3:]
        self.joint_command[3:] = joint_angles[3:]

    def js_cb(self, data):
        self.joint_states = np.array(data.position)

    def joint_command_loop(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.send_joint_command(*self.joint_command.tolist())
            rate.sleep()

    def send_goal(self, x, y, z, roll=0, pitch=0, yaw=0, frame="ur_arm_ee_link"):
        t_desired = np.array([[x],
                             [y],
                             [z]])
        if roll != 0 or pitch != 0 or yaw != 0:
            R_desired = T.euler_matrix(roll, pitch, yaw)[:3, :3]
        else:
            R_desired = np.eye(3)
        desired = np.block([[R_desired, t_desired], [0, 0, 0, 1]])

        state_transform = pose_lookup("ur_arm_starting_pose", frame)
        state = transform_stamped_to_se3(state_transform)

        final_state = np.matmul(state, desired)
        t_final = final_state[:, 3][:3]
        R_final = final_state[:3, :3]

        q = T.quaternion_from_matrix(final_state)
        # q = T.quaternion_from_matrix(R_final)

        ee_pose = Pose()
        ee_pose_goal = EEPoseGoals()
        ee_pose.position.x = float(t_final[0])
        ee_pose.position.y = float(t_final[1])
        ee_pose.position.z = float(t_final[2])
        ee_pose.orientation.w = float(q[0])
        ee_pose.orientation.x = float(q[1])
        ee_pose.orientation.y = float(q[2])
        ee_pose.orientation.z = float(q[3])

        ee_pose_goal.ee_poses.append(ee_pose)
        ee_pose_goal.header.seq = self.seq
        self.ee_pose_goals_pub.publish(ee_pose_goal)
        self.seq += 1

    def send_joint_command(self, elbow, lift, pan, wrist1, wrist2, wrist3):
        msg = JointTrajectory()
        point = JointTrajectoryPoint()
        point.positions = [elbow,
                           lift,
                           pan,
                           wrist1,
                           wrist2,
                           wrist3]
        point.velocities = []
        point.accelerations = []
        point.effort = []
        msg.points.append(point)
        msg.joint_names = ['ur_arm_elbow_joint',
                           'ur_arm_shoulder_lift_joint',
                           'ur_arm_shoulder_pan_joint',
                           'ur_arm_wrist_1_joint',
                           'ur_arm_wrist_2_joint',
                           'ur_arm_wrist_3_joint']

        msg.header.stamp = rospy.Time.now()  # TODO: maybe use seq
        try:
            error = np.abs(self.joint_command - self.joint_states)
        except:
            error = 5*np.ones(6)
        w = np.array([3, 3, 3, 0.2, 0.2, 0.2])
        # TODO: maybe should be matmul
        weight = np.abs(np.sum(np.multiply(w, error)))
        point.time_from_start = rospy.Duration(max(weight, 1.0))

        self.angular_pose_pub.publish(msg)

    def send_to_home(self):
        self.send_joint_command(*self.home)

    def initial_home(self):
        self.send_joint_command(*self.home)

    def clear_goal(self):
        blank = EEPoseGoals()
        blank_pose = Pose()
        blank.ee_poses.append(blank_pose)
        blank.header.seq = self.seq
        self.ee_pose_goals_pub.publish(blank)  # punch relaxed_ik
        self.seq += 1

    def send_transforms(self):
        broadcaster = StaticTransformBroadcaster()
        ee_link_pose = TransformStamped()
        flange_to_base = pose_lookup("ur_arm_base_link", "ur_arm_flange")
        if flange_to_base is not None:
            rot = flange_to_base.transform.rotation
            ee_link = transform(ee_link_pose, "ur_arm_flange",
                                "ur_arm_ee_link", rot.x, rot.y, rot.z, rot.w)
            starting_pose = transform(
                flange_to_base, "ur_arm_base_link", "ur_arm_starting_pose", 0, 0, 0)
        else:
            starting_pose = None
            rospy.logerr(
                "No transform found from ur_arm _flange to ur_arm_base_link.")

        if ee_link is not None and starting_pose is not None:
            broadcaster.sendTransform([ee_link, starting_pose])


def transform_stamped_to_se3(msg: TransformStamped):
    pose = msg.transform.translation
    rot = msg.transform.rotation
    t = np.array([[pose.x], [pose.y], [pose.z]])
    rot_matrix = T.quaternion_matrix([rot.w, rot.x, rot.y, rot.z])[:3, :3]
    se3 = np.block([[rot_matrix, t], [0, 0, 0, 1]])
    return se3

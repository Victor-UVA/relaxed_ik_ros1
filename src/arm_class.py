import threading
import rospy
from relaxed_ik_ros1.msg import EEPoseGoals
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from relaxed_ik_ros1.msg import JointAngles
from geometry_msgs.msg import Pose
import transformations as T
from tf_functions import pose_lookup, transform
import numpy as np
from se3_conversion import msg_to_se3
from tf2_ros import StaticTransformBroadcaster


class Arm:
    def __init__(self, home):
        self.ee_pose_goals_pub = rospy.Publisher(
            '/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=10)
        self.angular_pose_pub = rospy.Publisher(
            "ur/ur_arm_scaled_pos_joint_traj_controller/command", JointTrajectory, queue_size=10)
        self.joint_states_sub = rospy.Subscriber(
            "joint_states", JointState, self.js_cb)
        self.rleaxed_ik_joint_angles = rospy.Subscriber(
            "relaxed_ik/joint_angle_solutions", JointAngles, self.rik_ja_cb)

        self.joint_states = np.zeros(6)
        self.init_state = np.zeros(6)
        self.joint_command = np.array(home)

        self.seq = 0
        self.home = home

        # UR Interface
        rospy.loginfo("Initializing UR Interface...")
        rospy.sleep(5)
        rospy.loginfo("Homing...")
        self.send_to_home()
        rospy.sleep(8)
        self.send_transforms()
        rospy.sleep(2)
        rospy.loginfo("Ready to begin tasks")

        thread_loop = threading.Thread(target=self.joint_command_loop)
        thread_loop.start()

    def rik_ja_cb(self, data):
        joint_angles = np.array(data.angles.data)
        self.joint_command[0:3] = np.flip(joint_angles)[3:]
        self.joint_command[3:] = joint_angles[3:]

    def js_cb(self, data):
        time = rospy.Time.now()
        self.joint_states = np.array(data.position)

    def joint_command_loop(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.send_joint_command(*self.joint_command.tolist())
            rate.sleep()

    # TODO: choose between global and local frames
    def send_goal(self, x, y, z, roll=0, pitch=0, yaw=0):
        t_desired = np.array([x],
                             [y],
                             [z])
        R_desired = T.euler_matrix(roll, pitch, yaw)  # np.eye(3)

        desired = np.block([R_desired, t_desired], [0, 0, 0, 1])

        state_transform = pose_lookup(
            "ur_arm_starting_pose", "ur_arm_ee_link")  # or ee_2_link?
        state = msg_to_se3(state_transform)

        final_state = np.matmul(state, desired)
        t_final = final_state[:, 3][:3]
        R_final = final_state[:3, :3]

        q = T.quaternion_from_matrix(R_final)

        ee_pose = Pose()
        ee_pose_goal = EEPoseGoals()
        ee_pose.position.x = float(t_final[0])
        ee_pose.position.y = float(t_final[1])
        ee_pose.position.z = float(t_final[2])
        ee_pose.orientation.w = float(q[3])
        ee_pose.orientation.x = float(q[0])
        ee_pose.orientation.y = float(q[1])
        ee_pose.orientation.z = float(q[2])

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
        point.time_from_start.secs = 5
        msg.points.append(point)
        msg.joint_names = ['ur_arm_elbow_joint',
                           'ur_arm_shoulder_lift_joint',
                           'ur_arm_shoulder_pan_joint',
                           'ur_arm_wrist_1_joint',
                           'ur_arm_wrist_2_joint',
                           'ur_arm_wrist_3_joint']
        msg.header.stamp.secs = 0
        error = np.abs(self.joint_command - self.joint_states)
        w = np.array([2, 2, 2, 0.2, 0.2, 0.2])
        # TODO: make sure that it should be matmul
        weight = np.abs(np.sum(np.matmul(w, error)))
        point.time_from_start = rospy.Duration(max(weight, 0.5))
        # rospy.set_param('/scaled_vel_joint_traj_controller/constraints/goal_time',5*int(weight))

        self.angular_pose_pub.publish(msg)

    def send_to_home(self):
        self.send_joint_command(*self.home)

    def send_transforms(self):
        broadcaster = StaticTransformBroadcaster()
        ee_link = transform(pose_lookup("ur_arm_wrist_2_link", "ur_arm_flange"),
                            "ur_arm_wrist_2_link", "ur_arm_ee_link", 0, -1.5707, 1.5707)
        starting_pose = transform(pose_lookup(
            "ur_arm_base_link", "ur_arm_flange"), "ur_arm_base_link", "ur_arm_starting_pose", 0, -1.5707, 3.14159)
        if ee_link is not None and starting_pose is not None:
            broadcaster.sendTransform([ee_link, starting_pose])

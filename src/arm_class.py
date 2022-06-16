import rospy
from relaxed_ik_ros1.msg import EEPoseGoals
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
import transformations as T
import tf_functions
import numpy as np
from se3_conversion import msg_to_se3


class Arm:
    def __init__(self, home):
        self.ee_pose_goals_pub = rospy.Publisher(
            '/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=10)
        self.angular_pose_pub = rospy.Publisher(
            "ur/ur_arm_scaled_pos_joint_traj_controller/command", JointTrajectory, queue_size=10)

        self.x = 0
        self.y = 0
        self.y = 0
        self.seq = 0
        self.home = home
        # UR Interface
        # rospy.loginfo("Initializing UR Interface...")
        # self.ee = end_effector(home)
        # rospy.sleep(5)
        # rospy.loginfo("Homing...")
        # self.ee.send_to_home()
        # rospy.sleep(8)
        # self.ee.send_transforms()
        # rospy.sleep(2)
        # rospy.loginfo("Ready to begin tasks")

    # TODO: choose between global and local frames
    def send_goal(self, x, y, z, roll=0, pitch=0, yaw=0):
        t_desired = np.array([x],
                             [y],
                             [z])
        R_desired = T.euler_matrix(roll, pitch, yaw)  # np.eye(3)

        desired = np.block([R_desired, t_desired], [0, 0, 0, 1])

        state_transform = tf_functions.pose_lookup(
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

    def send_joint_angles(self, elbow, lift, pan, wrist1, wrist2, wrist3):
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
        self.angular_pose_pub.publish(msg)

    def send_home(self):
        self.send_joint_angles(*self.home)

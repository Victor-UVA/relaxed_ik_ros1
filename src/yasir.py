#! /usr/bin/env python3

import math

from matplotlib.ft2font import HORIZONTAL
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
import threading
import yaml
import rospkg
from sensor_msgs.msg import Joy
from enum import Enum, auto
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
    HOME = auto()
    HORIZONTAL = auto()
    HIDDEN = auto()
    HIGH = auto()
    ROTATE_HOME = auto()
    ROTATE_HOME_C = auto()
    STAY = auto()
    HORIZONTALMID = auto()

home = [87.75,-121.05,156.76,144.78,-88.07,-181.91]
horizontal = [170.37, -13.94, 78.56, 197.83, -88.11, -180.91]
horizontalmid = [180.66,-78.00,121.49,197.07,-88.13,-180.91]
hidden =[138.92,-20.35,16.05,224.77,-88.13,-180.91]
high = [348.35, -54.53, 53.44, 177.91, -77.27, -180.45]
rotate_home = [87.75,-121.05,156.76,144.78,-88.07,-181.91]
state = States.HOME 




class Arm:
    def __init__(self, config_file_name: str, is_full_path_to_file=False):
        self.seq = 0
        
        if not is_full_path_to_file:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('relaxed_ik_ros1')
            self.path = str(pkg_path)+'/relaxed_ik_core/config/info_files/' + \
                str(config_file_name)
        else:
            self.path = config_file_name

        self.home = np.zeros(6)
        try:
            with open(self.path) as f:
                file = yaml.safe_load(f)
                starting_config = np.array(file['starting_config'])
                self.home[:3] = np.flip(starting_config[:3])
                self.home[3:] = starting_config[3:]
                rospy.loginfo("Home is set to angles: "+str(self.home))
        except:
            rospy.logerr("Could not open file at: " +
                         str(self.path)+" or has missing/invalid starting_config.")
            rospy.logerr("Home will be set to all zeros.")

        self.joint_states = np.zeros(6)
        self.init_state = np.zeros(6)
        self.joint_command = self.home.copy()
        self.listen_to_rik = False

        self.ee_pose_goals_pub = rospy.Publisher(
            '/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=10)
        self.angular_pose_pub = rospy.Publisher(
            "/scaled_pos_joint_traj_controller/command", JointTrajectory, queue_size=10)
        self.joint_states_sub = rospy.Subscriber(
            "/joint_states", JointState, self.js_cb)
        self.relaxed_ik_joint_angles = rospy.Subscriber(
            "/relaxed_ik/joint_angle_solutions", JointAngles, self.rik_ja_cb)

        # UR Interface
        rospy.loginfo("Initializing UR Interface...")
        rospy.sleep(1)
        rospy.loginfo("Homing...")
        self.send_to_home()
        # wait until reaches home
        rospy.sleep(15)
        rospy.loginfo("Sending transforms")
        # self.send_transforms()
        rospy.sleep(1)
        rospy.loginfo("Ready to begin tasks")

        thread_loop = threading.Thread(target=self.joint_command_loop)
        thread_loop.start()

    def rik_ja_cb(self, data):
        if self.listen_to_rik:
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

    def send_goal(self, x, y, z, roll=0, pitch=0, yaw=0, frame="ee_link"):
        t_desired = np.array([[x],
                             [y],
                             [z]])
        if roll != 0 or pitch != 0 or yaw != 0:
            R_desired = T.euler_matrix(roll, pitch, yaw)[:3, :3]
        else:
            R_desired = np.eye(3)
        desired = np.block([[R_desired, t_desired], [0, 0, 0, 1]])

        state_transform = pose_lookup("starting_pose", frame)
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
        self.listen_to_rik = True

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
        msg.joint_names = ['elbow_joint',
                           'shoulder_lift_joint',
                           'shoulder_pan_joint',
                           'wrist_1_joint',
                           'wrist_2_joint',
                           'wrist_3_joint']

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

    def set_as_home(self, save_to_config_file=False):
        self.home = self.joint_command.copy()

        if save_to_config_file:
            new_home = np.zeros(6)
            new_home[:3] = np.flip(self.home)[:3]
            new_home[3:] = self.home[3:]
            with open(self.path) as f:
                file = yaml.safe_load(f)
            file['starting_config'] = new_home
            with open(self.path, 'w') as f:
                yaml.dump(file, f)

    def send_to_home(self):
        self.listen_to_rik = False
        self.joint_command = self.home.copy()
        self.send_joint_command(*self.home.tolist())
    
    def set_joint_command(self, command):
        self.listen_to_rik = False
        self.joint_command = np.array(command)
    
    def send_transforms(self):
        broadcaster = StaticTransformBroadcaster()
        ee_link_pose = TransformStamped()
        flange_to_base = pose_lookup("base_link", "flange")
        base_rot = pose_lookup("flange", "base_link")
        if flange_to_base is not None:
            # rot = flange_to_base.transform.rotation
            rot = base_rot.transform.rotation
            ee_link = transform(ee_link_pose, "flange",
                                "ee_link", rot.x, rot.y, rot.z, rot.w)
            starting_pose = transform(
                flange_to_base, "base_link", "starting_pose", 0, 0, 0)
        else:
            starting_pose = None
            rospy.logerr(
                "No transform found from ur_arm _flange to base_link.")

        if ee_link is not None and starting_pose is not None:
            broadcaster.sendTransform([ee_link, starting_pose])

def joyCB(data):
    global state
    if data.buttons[0] == 1:
        print("homeJCB")
        state = States.HOME 
    elif data.buttons[1] == 1:
        print("horizontalJCB")
        state = States.HORIZONTAL
    elif data.buttons[2] == 1:
        print("hiddenJCB")
        state = States.HIDDEN
    elif data.buttons[3] == 1:
        print("highJCB")
        state = States.HIGH
    elif data.axes[6] == 1:
        print("rotate_homeJCB")
        state = States.ROTATE_HOME
    elif data.axes[6] == -1.0:
        print("rotate_homeJCB clockwise")
        state = States.ROTATE_HOME_C
    elif data.buttons[4] == 1: #left bumper
        print("horizontal mid")
        state = States.HORIZONTALMID


def degree_to_radian(degree):
    return degree * (math.pi/180)

def rotate_counterclockwise():
    rotate_home[0] = rotate_home[0] + 90.0
    return

def rotate_clockwise():
    rotate_home[0] = rotate_home[0] - 90.0
    return


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

def transform_stamped_to_se3(msg: TransformStamped):
    pose = msg.transform.translation
    rot = msg.transform.rotation
    t = np.array([[pose.x], [pose.y], [pose.z]])
    rot_matrix = T.quaternion_matrix([rot.w, rot.x, rot.y, rot.z])[:3, :3]
    se3 = np.block([[rot_matrix, t], [0, 0, 0, 1]])
    return se3


if __name__ == "__main__":
    rospy.init_node('open_door')
    rospy.Subscriber("/joy", Joy, joyCB)
    arm = Arm(config_file_name="ur5_info.yaml")
    rate = rospy.Rate(5)
    elbow = degree_to_radian(home[0])
    lift = degree_to_radian(home[1])
    pan = degree_to_radian(home[2])
    wrist1 = degree_to_radian(home[3])
    wrist2 = degree_to_radian(home[4])
    wrist3 = degree_to_radian(home[5])
    
    while not rospy.is_shutdown():
        #get input from controller
        if state == States.HOME:
            rospy.loginfo("home")
            elbow = degree_to_radian(home[0])
            lift = degree_to_radian(home[1])
            pan = degree_to_radian(home[2])
            wrist1 = degree_to_radian(home[3])
            wrist2 = degree_to_radian(home[4])
            wrist3 = degree_to_radian(home[5])
            
        elif state == States.HORIZONTAL:
            rospy.loginfo("horizontal")
            elbow = degree_to_radian(horizontal[0])
            lift = degree_to_radian(horizontal[1])
            pan = degree_to_radian(horizontal[2])
            wrist1 = degree_to_radian(horizontal[3])
            wrist2 = degree_to_radian(horizontal[4])
            wrist3 = degree_to_radian(horizontal[5])

        elif state == States.HIDDEN:
            rospy.loginfo("hidden")
            elbow = degree_to_radian(hidden[0])
            lift = degree_to_radian(hidden[1])
            pan = degree_to_radian(hidden[2])
            wrist1 = degree_to_radian(hidden[3])
            wrist2 = degree_to_radian(hidden[4])
            wrist3 = degree_to_radian(hidden[5])

        elif state == States.HIGH:
            rospy.loginfo("high")
            elbow = degree_to_radian(high[0])
            lift = degree_to_radian(high[1])
            pan = degree_to_radian(high[2])
            wrist1 = degree_to_radian(high[3])
            wrist2 = degree_to_radian(high[4])
            wrist3 = degree_to_radian(high[5])
        elif state == States.ROTATE_HOME:
            rospy.loginfo("rotate counterclockwise")
            rotate_counterclockwise()
            elbow = degree_to_radian(rotate_home[0])
            lift = degree_to_radian(rotate_home[1])
            pan = degree_to_radian(rotate_home[2])
            wrist1 = degree_to_radian(rotate_home[3])
            wrist2 = degree_to_radian(rotate_home[4])
            wrist3 = degree_to_radian(rotate_home[5])
            state = States.STAY
        elif state == States.ROTATE_HOME_C:
            rospy.loginfo("rotate clockwise")
            rotate_clockwise()
            elbow = degree_to_radian(rotate_home[0])
            lift = degree_to_radian(rotate_home[1])
            pan = degree_to_radian(rotate_home[2])
            wrist1 = degree_to_radian(rotate_home[3])
            wrist2 = degree_to_radian(rotate_home[4])
            wrist3 = degree_to_radian(rotate_home[5])
            state = States.STAY
        elif state == States.HORIZONTALMID:
            rospy.loginfo("horizontal mid")
            elbow = degree_to_radian(horizontalmid[0])
            lift = degree_to_radian(horizontalmid[1])
            pan = degree_to_radian(horizontalmid[2])
            wrist1 = degree_to_radian(horizontalmid[3])
            wrist2 = degree_to_radian(horizontalmid[4])
            wrist3 = degree_to_radian(horizontalmid[5])
        elif state == States.STAY:
            rospy.loginfo("stay")

        command = np.array([elbow, lift, pan, wrist1, wrist2, wrist3])
        og_command = np.array(command)
        command[:3] = np.flip(og_command[:3])
        command[3:] = og_command[3:]

        arm.set_joint_command(command)
        print("elbow" , elbow)
        print("lift" , lift)
        print("pan" , pan)
        print("wrist1" , wrist1)
        print("wrist2" , wrist2)
        print("wrist3" , wrist3)
        print("------------------------")
        rospy.sleep(5)
        # rate.sleep()

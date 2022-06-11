#! /usr/bin/env python3

from typing import Sequence
import numpy as np
import rospy
from relaxed_ik_ros1.msg import EEPoseGoals
# from relaxed_ik_node import EEPoseGoals
from geometry_msgs.msg import Pose, PoseArray
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped
import tf
import tf_conversions
import readchar
import ur_interface
import transformations as T

class door_manipulation():
    def __init__(self):
        self.ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=10)
        # self.current_ar_tag_pose_sub = rospy.Subscriber('/ar_tag_pos', PoseArray,self. _ar_tag_pos_cb)
        # self.num_ar_tags = 0
        # self.id_choice = 0
        # self.marker_info = None
        self.manipulator_radius = .9

        self.distance_from_doorknob = [0,.41,.125] # <--- x, y, z distance from doorknob center - measured from tip of attachment - will soon be taken from perception data
        self.doorknob_radius = .1 # <--- distance from lock on doorknob to bend of handle
        self.seq = 1
        self.pos_stride = 0.015
        self.rot_stride = 0.5

        self.position_r = [0,0,0]
        self.rotation_r = [1,0,0,0]

        self.position_l = [0,0,0]
        self.rotation_l = [1,0,0,0]

    def read_key(self):
        print("Pos R: {}, Rot R: {}".format(self.position_r, self.rotation_r))

        key = readchar.readkey()
        if key == 'w':
            self.position_r[0] += self.pos_stride
        elif key == 'x':
            self.position_r[0] -= self.pos_stride
        elif key == 'a':
            self.position_r[1] += self.pos_stride
        elif key == 'd':
            self.position_r[1] -= self.pos_stride
        elif key == 'q':
            self.position_r[2] += self.pos_stride
        elif key == 'z':
            self.position_r[2] -= self.pos_stride
        elif key == '1':
            euler = list(T.euler_from_quaternion(self.rotation_r))
            euler[0] += self.rot_stride
            self.rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '2':
            euler = list(T.euler_from_quaternion(self.rotation_r))
            euler[0] -= self.rot_stride
            self.rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '3':
            euler = list(T.euler_from_quaternion(self.rotation_r))
            euler[1] += self.rot_stride
            self.rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '4':
            euler = list(T.euler_from_quaternion(self.rotation_r))
            euler[1] -= self.rot_stride
            self.rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '5':
            euler = list(T.euler_from_quaternion(self.rotation_r))
            euler[2] += self.rot_stride
            self.rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        elif key == '6':
            euler = list(T.euler_from_quaternion(self.rotation_r))
            euler[2] -= self.rot_stride
            self.rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])

        elif key == 'h':
            self.home()
        elif key == 'p':
            self.approach_doorknob()
        elif key == 'g':
            self.grab_handle()
        elif key == 't':
            self.twist_handle(55)
        elif key == 'u':
            self.untwist_handle(55)
        elif key == 'o':
            self.open_first(10)
        # elif key == 'c':
        #     self.circle_point_xz(45,self.doorknob_radius)

        elif key == 'c':
            rospy.signal_shutdown()
        
        self.go()

    def home(self):
        print("going home")
        self.position_r = [0,0,0]
        self.rotation_r = [1,0,0,0]

        self.position_l = [0,0,0]
        self.rotation_l = [1,0,0,0]
        self.go()

    def approach_doorknob(self):
        print("approaching doorknob")
        self.position_r[0] = self.distance_from_doorknob[0] + 3/4*self.doorknob_radius # approach to 3/4 of the length of the handle
        self.position_r[1] = self.distance_from_doorknob[1] + .045
        self.position_r[2] = self.distance_from_doorknob[2] + .105
        self.rotation_r = [1,0,0,0]

        self.position_l = [0,0,0]
        self.rotation_l = [1,0,0,0]

        self.go()

    def grab_handle(self):
        print("grabbing handle")
        self.position_r[2] = .125
        self.go()

    def circle_point_xz(self,theta,r):
        # could calculate r instead using current x and z with center x and z
        theta_radians = np.math.pi/180*theta

        dtheta = theta_radians/5
        count = 1
        print("turning handle ", dtheta, " degrees")
        while(count*dtheta<=theta_radians):  
            dx = r*np.math.sin(dtheta)
            dz = r*(1-np.math.cos(dtheta))
            self.position_r[0] += dx
            self.position_r[2] += dz

            euler = list(T.euler_from_quaternion(self.rotation_r))
            euler[2] += self.rot_stride
            self.rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        
            self.go()
            count += count
            rospy.sleep(.5)


    def twist_handle(self,theta_degrees):
        theta_radians = np.math.pi/180*theta_degrees

        dtheta = theta_radians
        count = 1
        print("turning handle ", dtheta, " degrees")
        while(count*dtheta<=theta_radians):
            
            dx = 3/4*self.doorknob_radius*(1-np.math.cos(dtheta))
            dy = 3/4*self.doorknob_radius*np.math.sin(dtheta)
            self.position_r[2] = self.position_r[2] - dy # dy based on angle
            self.position_r[0] = self.position_r[0] - dx # dx based on angle

            euler = list(T.euler_from_quaternion(self.rotation_r))
            euler[1] += self.rot_stride   # < ------ problem here!!!!
            self.rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])

            self.go()
            count += count
            rospy.sleep(.5)

    def untwist_handle(self,theta_degrees):
        theta_radians = np.math.pi/180*theta_degrees

        dtheta = theta_radians
        count = 1
        print("turning handle ", theta_degrees, " degrees")
        while(count*dtheta<=theta_radians):
            
            dx = 3/4*self.doorknob_radius*(1-np.math.cos(dtheta))
            dy = 3/4*self.doorknob_radius*np.math.sin(dtheta)
            self.position_r[2] = self.position_r[2] + dy # dy based on angle
            self.position_r[0] = self.position_r[0] + dx # dx based on angle

            euler = list(T.euler_from_quaternion(self.rotation_r))
            euler[1] -= self.rot_stride # < ------ problem here!!!!
            self.rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])

            self.go()
            count += count
            rospy.sleep(.5)

    def open_first(self,theta_degrees):
        theta_radians = np.math.pi/180*theta_degrees

        dtheta = theta_radians/2
        count = 1
        print("opening door ", theta_degrees, " degrees")
        while(count*dtheta<=theta_radians):
            
            dx = 3/4*self.doorknob_radius*(1-np.math.cos(dtheta))
            dz = 3/4*self.doorknob_radius*np.math.sin(dtheta)
            self.position_r[1] = self.position_r[1] + dz # dy based on angle
            self.position_r[0] = self.position_r[0] - dx # dx based on angle

            euler = list(T.euler_from_quaternion(self.rotation_r))
            euler[1] -= self.rot_stride  # < ------ problem here!!!!
            self.rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])

            self.go()
            count += count
            rospy.sleep(.5)

        

    def go(self):
        ee_pose_goals = EEPoseGoals()
        pose_r = Pose()
        pose_r.position.x = self.position_r[0]
        pose_r.position.y = self.position_r[1]
        pose_r.position.z = self.position_r[2]

        pose_r.orientation.w = self.rotation_r[0]
        pose_r.orientation.x = self.rotation_r[1]
        pose_r.orientation.y = self.rotation_r[2]
        pose_r.orientation.z = self.rotation_r[3]

        pose_l = Pose()
        pose_l.position.x = self.position_l[0]
        pose_l.position.y = self.position_l[1]
        pose_l.position.z = self.position_l[2]

        pose_l.orientation.w = self.rotation_l[0]
        pose_l.orientation.x = self.rotation_l[1]
        pose_l.orientation.y = self.rotation_l[2]
        pose_l.orientation.z = self.rotation_l[3]
        ee_pose_goals.ee_poses.append(pose_r)
        ee_pose_goals.ee_poses.append(pose_l)
        #ee_pose_goals.header.seq = seq
        #seq += 1
        self.ee_pose_goals_pub.publish(ee_pose_goals)



if __name__=='__main__':

    rospy.init_node('Door_manipulation')
    print('                ')
    print('ALERT!!! T-REX WILL MOVE NOW!!! PLEASE MAINTAIN DISTANCE :) You have 2 seconds')
    print('                ')
    rate = 2
    rospy.sleep(rate)
    robot_pose = door_manipulation()
    #rospy.sleep(2)
    rate = rospy.Rate(100)
    print(rate)
    while not rospy.is_shutdown():
        robot_pose.read_key()
        #rospy.spin
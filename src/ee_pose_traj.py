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

class ee_poses():
    def __init__(self):
        self.ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=10)
        # self.current_ar_tag_pose_sub = rospy.Subscriber('/ar_tag_pos', PoseArray,self. _ar_tag_pos_cb)
        # self.num_ar_tags = 0
        # self.id_choice = 0
        # self.marker_info = None
        self.seq = 1

    # def _ar_tag_pos_cb(self,data):
        # print(len(data.poses))
        # self.num_ar_tags = len(data.poses)
        # self.marker_info = data
        # print(self.marker_info)

    def init_motion(self):
        ee_pose = Pose()
        ee_pose_goal = EEPoseGoals()
        if self.seq == 1:
            ee_pose.position.x = 0
            ee_pose.position.y = 0
            ee_pose.position.z = 0
            ee_pose.orientation.w = 1
            ee_pose.orientation.x = 0
            ee_pose.orientation.y = 0
            ee_pose.orientation.z = 0

            ee_pose_goal.ee_poses.append(ee_pose)

            ee_pose_goal.header.seq = self.seq
            print('At Origin. Going to sleep for 2 seconds')
            self.ee_pose_goals_pub.publish(ee_pose_goal)
            self.seq += 1

            # rospy.sleep(2)

        else:
            if self.seq == 2:
                ee_pose.position.x = 0
                ee_pose.position.y = 0
                ee_pose.position.z = -0.05
                ee_pose.orientation.w = 1
                ee_pose.orientation.x = 0
                ee_pose.orientation.y = 0
                ee_pose.orientation.z = 0

                ee_pose_goal.ee_poses.append(ee_pose)

                ee_pose_goal.header.seq = self.seq
                print('Position: ', self.seq)
                self.ee_pose_goals_pub.publish(ee_pose_goal)
                self.seq += 1

            elif self.seq == 3:
                ee_pose.position.x = 0
                ee_pose.position.y = 0.10
                ee_pose.position.z = 0.05
                ee_pose.orientation.w = 1
                ee_pose.orientation.x = 0
                ee_pose.orientation.y = 0
                ee_pose.orientation.z = 0

                ee_pose_goal.ee_poses.append(ee_pose)

                ee_pose_goal.header.seq = self.seq
                print('Position: ', self.seq)
                self.ee_pose_goals_pub.publish(ee_pose_goal)
                self.seq += 1

            elif self.seq == 4:
                ee_pose.position.x = 0
                ee_pose.position.y = 0.10
                ee_pose.position.z = 0.15
                q = tf_conversions.transformations.quaternion_from_euler(0,0,0)
                ee_pose.orientation.w = q[3]
                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]

                ee_pose_goal.ee_poses.append(ee_pose)

                ee_pose_goal.header.seq = self.seq
                print('Position: ', self.seq)
                self.ee_pose_goals_pub.publish(ee_pose_goal)
                self.seq += 1

            elif self.seq == 5:
                ee_pose.position.x = -0.3
                ee_pose.position.y = 0.74
                ee_pose.position.z = 0.135
                q = tf_conversions.transformations.quaternion_from_euler(0,0,1.5707)
                ee_pose.orientation.w = q[3]
                ee_pose.orientation.x = q[0]
                ee_pose.orientation.y = q[1]
                ee_pose.orientation.z = q[2]

                ee_pose_goal.ee_poses.append(ee_pose)

                ee_pose_goal.header.seq = self.seq
                print('Position: ', self.seq)
                self.ee_pose_goals_pub.publish(ee_pose_goal)
                self.seq += 1
            
            elif self.seq == 6:
                self.push_button()
                self.unpush_button()
                self.seq += 1
            elif self.seq ==7:
                self.flip_up()
                self.unflip_up()
                self.seq += 1
            elif self.seq ==8:
                self.flip_down()
                self.unflip_down()
                self.seq += 1
            elif self.seq ==9:
                self.flip_left()
                self.unflip_left()
                self.seq += 1
            elif self.seq ==10:
                self.flip_right()
                self.unflip_right()
                self.seq += 1
            # else: 
                # self.go_to_ar_tag()

    # def go_to_ar_tag(self):
        # ee_pose = Pose()
        # ee_pose_goal = EEPoseGoals()
        # print(self.marker_info.poses[1])
        # ee_pose.position.x = -self.marker_info.poses[1].position.x + 0.22 - 0.1
        # ee_pose.position.y = -self.marker_info.poses[1].position.y + 0.095
        # ee_pose.position.z = self.marker_info.poses[1].position.z - 0.055
        # ee_pose.orientation.w = 1
        # ee_pose.orientation.x = 0
        # ee_pose.orientation.y = 0
        # ee_pose.orientation.z = 0
        # ee_pose_goal.ee_poses.append(ee_pose)
        # ee_pose_goal.header.seq = self.seq
        # print('Going to AR Tag')
        # self.ee_pose_goals_pub.publish(ee_pose_goal)
        # self.seq += 1
    
    def home(self):
        print('Home')

    def push_button(self):
        print('Push Button')

        self.goal_pose(0.1,0.0,0.0,0.0,0.0,0.0)
        self.primitive_commands()

    def unpush_button(self):
        print('Unpush Button')

        self.goal_pose(-0.1,0.0,0.0,0.0,0.0,0.0)
        self.primitive_commands()

    def flip_up(self):
        print('Flip Up')

        self.flip()

        self.goal_pose(0.0,0.0,0.1,0.0,0.0,0.0)
        self.primitive_commands()

    def unflip_up(self):
        print('Unflip Up')

        self.goal_pose(0.0,0.0,-0.1,0.0,0.0,0.0)
        self.primitive_commands()

        self.unflip()
    
    def flip_down(self):
        print('Flip Down')

        self.flip()

        self.goal_pose(0.0,0.0,-0.1,0.0,0.0,0.0)
        self.primitive_commands()

    def unflip_down(self):
        print('Unflip Down')

        self.goal_pose(0.0,0.0,0.1,0.0,0.0,0.0)
        self.primitive_commands()

        self.unflip()
    
    def flip_left(self):
        print('Flip Left')

        self.flip()

        self.goal_pose(0.0,0.1,0.0,0.0,0.0,0.0)
        self.primitive_commands()

    def unflip_left(self):
        print('Unflip Left')

        self.goal_pose(0.0,-0.1,0.0,0.0,0.0,0.0)
        self.primitive_commands()

        self.unflip()

    def flip_right(self):
        print('Flip Right')

        self.flip()

        self.goal_pose(0.0,-0.1,0.0,0.0,0.0,0.0)
        self.primitive_commands()

    def unflip_right(self):
        print('Unflip Right')

        self.goal_pose(0.0,0.1,0.0,0.0,0.0,0.0)
        self.primitive_commands()

        self.unflip()

    def flip(self):
        self.goal_pose(0.1,0.0,0.0,0.0,0.0,0.0)
        self.primitive_commands()
    
    def unflip(self):
        self.goal_pose(-0.1,0.0,0.0,0.0,0.0,0.0)
        self.primitive_commands()

    def grab_bag(self):
        print('Grab Bag')

        self.goal_pose(0.1,0.0,0.0,0.0,0.0,0.0)
        self.primitive_commands()

        self.goal_pose(0.0,0.0,0.0,-1.5707,0.0,0.0)
        self.primitive_commands()

        self.goal_pose(0.0,0.0,0.1,-1.5707,0.0,0.0)
        self.primitive_commands()

    def ungrab_bag(self):
        print('Ungrab Bag')

        self.goal_pose(0.0,0.0,-0.1,-1.5707,0.0,0.0)
        self.primitive_commands()

        self.goal_pose(0.0,0.0,0.0,0.0,0.0,0.0)
        self.primitive_commands()

        self.goal_pose(-0.1,0.0,0.0,0.0,0.0,0.0)
        self.primitive_commands()
    
    def move_bag(self,x_pos,y_pos,z_pos,z_rot):
        print('Move Bag')

        ee_pose = Pose()
        ee_pose_goal = EEPoseGoals()

        ee_pose.position.x = x_pos
        ee_pose.position.y = y_pos
        ee_pose.position.z = z_pos

        q = tf_conversions.transformations.quaternion_from_euler(-1.5707,0,z_rot)

        ee_pose.orientation.x = float(q[0])
        ee_pose.orientation.y = float(q[1])
        ee_pose.orientation.z = float(q[2])
        ee_pose.orientation.w = float(q[3])

        ee_pose_goal.ee_poses.append(ee_pose)

        ee_pose_goal.header.seq = self.seq

        self.ee_pose_goals_pub.publish(ee_pose_goal)

    def primitive_commands(self):
        trans_goal = self.pose_lookup('starting_pose','goal_pose')
        # print(trans_goal)

        if trans_goal is not None:
            ee_pose = Pose()
            ee_pose_goal = EEPoseGoals()

            ee_pose.position.x = float(trans_goal.transform.translation.x)
            ee_pose.position.y = float(trans_goal.transform.translation.y)
            ee_pose.position.z = float(trans_goal.transform.translation.z)
            
            ee_pose.orientation.w = float(trans_goal.transform.rotation.w)
            ee_pose.orientation.x = float(trans_goal.transform.rotation.x)
            ee_pose.orientation.y = float(trans_goal.transform.rotation.y)
            ee_pose.orientation.z = float(trans_goal.transform.rotation.z)

            ee_pose_goal.ee_poses.append(ee_pose)

            ee_pose_goal.header.seq = self.seq
            
            self.ee_pose_goals_pub.publish(ee_pose_goal)
        
        rospy.sleep(0.5)

    def goal_pose(self,x_val,y_val,z_val,x_rot,y_rot,z_rot):
        br = tf2_ros.StaticTransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "ee_link"
        t.child_frame_id = "goal_pose"

        t.transform.translation.x = float(x_val)
        t.transform.translation.y = float(y_val)
        t.transform.translation.z = float(z_val)
        
        q = tf_conversions.transformations.quaternion_from_euler(x_rot,y_rot,z_rot)

        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])

        br.sendTransform(t)

    def pose_lookup(self,parent, child):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        rospy.sleep(2)
        
        try:
            trans =  tfBuffer.lookup_transform(parent, child, rospy.Time())
            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(5)

    def keyboard_control(self):
        print('Enter Command')
        key = readchar.readkey()
        if key == 'c':
            rospy.signal_shutdown()
        elif key == 'q':
            self.push_button()
        elif key == 'w':
            self.flip_up()
        elif key == 'e':
            self.flip_right()
        elif key == 'h':
            ur_interface.end_effector.send_to_home(ur_interface.ee)

    def initial_position(self):
        ee_pose = Pose()
        ee_pose_goal = EEPoseGoals()
        
        ee_pose.position.x = 0.60
        ee_pose.position.y = 0.40
        ee_pose.position.z = -0.29
        ee_pose.orientation.w = 1
        ee_pose.orientation.x = 0
        ee_pose.orientation.y = 0
        ee_pose.orientation.z = 0

        ee_pose_goal.ee_poses.append(ee_pose)

        ee_pose_goal.header.seq = self.seq
        print('Initial Position')
        self.ee_pose_goals_pub.publish(ee_pose_goal)

    def mid_position(self):
        ee_pose = Pose()
        ee_pose_goal = EEPoseGoals()

        ee_pose.position.x = 0.0
        ee_pose.position.y = 1.0
        ee_pose.position.z = 0.0

        q = tf_conversions.transformations.quaternion_from_euler(-1.5707,0,1.5707)

        ee_pose.orientation.x = float(q[0])
        ee_pose.orientation.y = float(q[1])
        ee_pose.orientation.z = float(q[2])
        ee_pose.orientation.w = float(q[3])

        ee_pose_goal.ee_poses.append(ee_pose)

        ee_pose_goal.header.seq = self.seq
        print('Midway Position')
        self.ee_pose_goals_pub.publish(ee_pose_goal)

    def final_position(self):
        ee_pose = Pose()
        ee_pose_goal = EEPoseGoals()

        ee_pose.position.x = -1.3
        ee_pose.position.y = 1.0
        ee_pose.position.z = -0.05

        q = tf_conversions.transformations.quaternion_from_euler(-1.5707,0,2.355)

        ee_pose.orientation.x = float(q[0])
        ee_pose.orientation.y = float(q[1])
        ee_pose.orientation.z = float(q[2])
        ee_pose.orientation.w = float(q[3])

        ee_pose_goal.ee_poses.append(ee_pose)

        ee_pose_goal.header.seq = self.seq
        print('Final Position')
        self.ee_pose_goals_pub.publish(ee_pose_goal)
    
    def pick_and_drop(self):
        self.initial_position()
        rospy.sleep(5)
        self.grab_bag()
        rospy.sleep(2)
        self.mid_position()
        rospy.sleep(4)
        self.final_position()
        rospy.sleep(8)
        self.ungrab_bag()
        rospy.sleep(5)
        self.home()
        rospy.sleep(5)

if __name__=='__main__':

    rospy.init_node('EE_pose_trajectrory')
    print('                ')
    print('ALERT!!! T-REX WILL MOVE NOW!!! PLEASE MAINTAIN DISTANCE :) You have 5 seconds')
    print('                ')
    rate = 5
    rospy.sleep(rate)
    # ee_pose_goals_pub = rospy.Publisher('/trex/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=10)
    #print('Sending Trajectory')
    robot_pose = ee_poses()
    # t = 0
    rospy.sleep(2)
    while not rospy.is_shutdown():
        # robot_pose.init_motion()
        # robot_pose.pick_and_drop()
        robot_pose.keyboard_control()
        # squareTraj(ee_pose_goals_pub)
        rospy.sleep(rate)

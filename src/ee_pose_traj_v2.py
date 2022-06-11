#! /usr/bin/env python3

from typing import Sequence
import numpy as np
import rospy
from relaxed_ik_ros1.msg import EEPoseGoals
#from relaxed_ik_node import EEPoseGoals
from geometry_msgs.msg import Pose, PoseArray
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped, PoseWithCovariance
import tf
import tf_conversions
import readchar
from ur_interface import end_effector

# tf_prefix = rospy.get_param("/ur_hardware_interface/tf_prefix")

# tf_prefix = rospy.get_param("/UR_Dingo/ur_hardware_interface/tf_prefix")

class ee_poses():
    def __init__(self):
        self.ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=10)
        # self.current_ar_tag_pose_sub = rospy.Subscriber('/ar_tag_pos', PoseArray,self. _ar_tag_pos_cb)
        # self.num_ar_tags = 0
        # self.id_choice = 0
        # self.marker_info = None
        self.seq = 1
        self.ee = end_effector()

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
        ee_pose = Pose()
        ee_pose_goal = EEPoseGoals()
        ee_pose.position.x = 0
        ee_pose.position.y = 0
        ee_pose.position.z = 0
        ee_pose.orientation.w = 1
        ee_pose.orientation.x = 0
        ee_pose.orientation.y = 0
        ee_pose.orientation.z = 0

        ee_pose_goal.ee_poses.append(ee_pose)

        ee_pose_goal.header.seq = self.seq
        print('At Origin.')
        self.ee_pose_goals_pub.publish(ee_pose_goal)

    def push_button(self):
        print('Push Button')

        self.goal_pose(0.1,0.0,0.0)
        self.primitive_commands()

    def unpush_button(self):
        print('Unpush Button')

        self.goal_pose(-0.1,0.0,0.0)
        self.primitive_commands()

    def flip_up(self):
        print('Flip Up')

        self.flip()

        self.goal_pose(0.0,0.0,0.1)
        self.primitive_commands()

    def unflip_up(self):
        print('Unflip Up')

        self.goal_pose(0.0,0.0,-0.1)
        self.primitive_commands()

        self.unflip()
    
    def flip_down(self):
        print('Flip Down')

        self.flip()

        self.goal_pose(0.0,0.0,-0.1)
        self.primitive_commands()

    def unflip_down(self):
        print('Unflip Down')

        self.goal_pose(0.0,0.0,0.1)
        self.primitive_commands()

        self.unflip()
    
    def flip_left(self):
        print('Flip Left')

        self.flip()

        self.goal_pose(0.0,0.1,0.0)
        self.primitive_commands()

    def unflip_left(self):
        print('Unflip Left')

        self.goal_pose(0.0,-0.1,0.0)
        self.primitive_commands()

        self.unflip()

    def flip_right(self):
        print('Flip Right')

        self.flip()

        self.goal_pose(0.0,-0.1,0.0)
        self.primitive_commands()

    def unflip_right(self):
        print('Unflip Right')

        self.goal_pose(0.0,0.1,0.0)
        self.primitive_commands()

        self.unflip()

    def flip(self):
        self.goal_pose(0.1,0.0,0.0)
        self.primitive_commands()
    
    def unflip(self):
        self.goal_pose(-0.1,0.0,0.0)
        self.primitive_commands()

    def clockwise_circle_from_edge_constant_x(self):
        print('Clockwise Circle from Edge Constant x')

        x = 0.0
        radius = 0.1
        step_size = 0.01

        for y in range(int(-radius*100),int(step_size*100),int(radius*100)):
            z = np.abs(np.sqrt(np.square(radius)-np.square(float(y/100))))
            print(x,y,z)
            self.goal_pose(x,y,z)
            self.primitive_commands()
        for y in range(int(radius*100),int(step_size*100),int(-radius*100)):
            z = -np.abs(np.sqrt(np.square(radius)-np.square(float(y/100))))
            self.goal_pose(x,y,z)
            self.primitive_commands()
    
    def clockwise_circle_from_center_constant_x(self):
        x = 0.0
        radius = 0.1
        step_size = 0.01

        self.goal_pose(z,-radius,0)
        self.primitive_commands

        for y in range(int(-radius*100),int(step_size*100),int(radius*100)):
            z = np.abs(np.sqrt(np.square(radius)-np.square(float(y/100))))
            print(x,y,z)
            self.goal_pose(x,y,z)
            self.primitive_commands()
        for y in range(int(radius*100),int(step_size*100),int(-radius*100)):
            z = -np.abs(np.sqrt(np.square(radius)-np.square(float(y/100))))
            self.goal_pose(x,y,z)
            self.primitive_commands()

    def primitive_commands(self):
        trans_goal = self.pose_lookup('starting_pose','goal_pose')
        
        #trans_goal = self.pose_lookup( tf_prefix + 'starting_pose', tf_prefix + 'goal_pose')
        # print(trans_goal)

        if trans_goal is not None:
            ee_pose = Pose()
            ee_pose_goal = EEPoseGoals()

            ee_pose.position.x = -float(trans_goal.transform.translation.x)
            ee_pose.position.y = -float(trans_goal.transform.translation.y)
            ee_pose.position.z = float(trans_goal.transform.translation.z)
            
            ee_pose.orientation.w = float(-trans_goal.transform.rotation.w)
            ee_pose.orientation.x = float(trans_goal.transform.rotation.x)
            ee_pose.orientation.y = float(trans_goal.transform.rotation.y)
            ee_pose.orientation.z = float(trans_goal.transform.rotation.z)

            ee_pose_goal.ee_poses.append(ee_pose)

            ee_pose_goal.header.seq = self.seq
            
            self.ee_pose_goals_pub.publish(ee_pose_goal)
        
        rospy.sleep(0.5)

    def goal_pose(self,x_val,y_val,z_val):
        br = tf2_ros.StaticTransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        # t.header.frame_id = tf_prefix + "ee_2_link"
        # t.child_frame_id = tf_prefix + "goal_pose"

        t.header.frame_id = "ee_2_link"
        t.child_frame_id = "goal_pose"

        t.transform.translation.x = float(x_val)
        t.transform.translation.y = float(y_val)
        t.transform.translation.z = float(z_val)
        
        q = tf_conversions.transformations.quaternion_from_euler(0,0,0)

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
        if key == 'b':
            self.push_button()
        elif key == 'v':
            self.unpush_button()
        # elif key == 'u':
        #     self.flip_up()
        # elif key == 'y':
        #     self.unflip_up()
        # elif key == 'd':
        #     self.flip_down()
        # elif key == 's':
        #     self.unflip_down()
        # elif key == 'l':
        #     self.flip_left()
        # elif key == 'k':
        #     self.unflip_left()
        # elif key == 'r':
        #     self.flip_right()
        # elif key == 'e':
        #     self.unflip_right()
        # elif key == 'h':
        #     self.home()
        elif key == 'c':
            rospy.signal_shutdown()
        elif key == 'p':
            self.panel_reach()
        elif key == 'a':
            self.near_switch()
        elif key == 'h':
            self.ee.send_to_home()

    def panel_reach(self):
        ee_pose = Pose()
        ee_pose_goal = EEPoseGoals()
        ee_pose.position.x = self.panel_x + 0.10
        ee_pose.position.y = self.panel_y
        ee_pose.position.z = self.panel_z
        ee_pose.orientation.w = 1
        ee_pose.orientation.x = 0
        ee_pose.orientation.y = 0
        ee_pose.orientation.z = 0

        ee_pose_goal.ee_poses.append(ee_pose)

        ee_pose_goal.header.seq = self.seq
        print('Panel Center')
        print(ee_pose_goal)
        self.ee_pose_goals_pub.publish(ee_pose_goal)

    def near_switch(self):
        print("Which switch do you want to press?")
        print("Please check the image window to see the numbers.")
        key = readchar.readkey()
        print(key)
        rospy.sleep(2)
        ee_pose = Pose()
        ee_pose_goal = EEPoseGoals()

        # switch_pose = self.pose_lookup(tf_prefix + "starting_pose","switch_"+str(key))

        switch_pose = self.pose_lookup("starting_pose","switch_"+str(key))

        # ee_pose.position.x = self.switch_x[int(key)] + 0.10
        # ee_pose.position.y = self.switch_y[int(key)] 
        # ee_pose.position.z = self.switch_z[int(key)] + 0.08
        # ee_pose.orientation.w = 1
        # ee_pose.orientation.x = 0
        # ee_pose.orientation.y = 0
        # ee_pose.orientation.z = 0

        ee_pose.position.x = -switch_pose.transform.translation.x + 0.165
        ee_pose.position.y = -switch_pose.transform.translation.y -0.03
        ee_pose.position.z = switch_pose.transform.translation.z + 0.07
        ee_pose.orientation.w = switch_pose.transform.rotation.w
        ee_pose.orientation.x = switch_pose.transform.rotation.x
        ee_pose.orientation.y = switch_pose.transform.rotation.y
        ee_pose.orientation.z = switch_pose.transform.rotation.z

        ee_pose_goal.ee_poses.append(ee_pose)
        ee_pose_goal.header.seq = self.seq
        print('Switch Center')
        print(ee_pose_goal)
        self.ee_pose_goals_pub.publish(ee_pose_goal)

    def go_to_panel_cb(self,data):
        self.panel_x = data.pose.position.x
        self.panel_y = data.pose.position.y
        self.panel_z = data.pose.position.z - 0.05

    def go_to_switch_cb(self,data):
        switches = data.poses
        self.switch_x = []
        self.switch_y = []
        self.switch_z = []
        self.num_switches = len(self.switch_x)
        for i in range(len(switches)):
            self.switch_x.append(switches[i].position.x)
            self.switch_y.append(-1*switches[i].position.z -0.05)
            self.switch_z.append(switches[i].position.y + 0.015)

        # print(self.switch_x)

    # def autonomous_button(self):
    #     # hit a, then 0, then b 
        
    #     self.near_switch()
    #     self.push_button()

# def callback(data):
#     value_A = data.buttons[1]
#     value_B = data.buttons[2]
#     if value_B == 1:
#         rosparam.set_param("run_arm", "false")
#     elif value_A == 1:
#         rosparam.set_param("run_arm", "true")


if __name__=='__main__':

    rospy.init_node('EE_pose_trajectrory')
    # rospy.Subscriber("joy_driving", Joy, callback)
    # rosparam.set_param("run_arm", "false")
    print('                ')
    print('ALERT!!! DINGO WILL MOVE NOW!!! PLEASE MAINTAIN DISTANCE :) You have 5 seconds')
    print('                ')
    rate = 5
    rospy.sleep(rate)
    ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=10)
    
    print('Sending Trajectory')
    robot_pose = ee_poses()
    ee_panels_sub = rospy.Subscriber('Panel_center_xyz',PoseWithCovariance,robot_pose.go_to_panel_cb)
    ee_switches_sub = rospy.Subscriber('Switch_xyz',PoseArray,robot_pose.go_to_switch_cb)
    # t = 0
    rospy.sleep(2)
    while not rospy.is_shutdown():
        # robot_pose.init_motion()
        robot_pose.keyboard_control()
        # squareTraj(ee_pose_goals_pub)
        rospy.sleep(rate)

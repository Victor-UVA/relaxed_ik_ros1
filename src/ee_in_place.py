#!/usr/bin/python3

import rospy
from arm_class import Arm

if __name__ == "__main__":
    rospy.init_node("ee_in_place")
    rate = rospy.Rate(5)
    arm = Arm(config_file_name="ur5e_info.yaml")
    while not rospy.is_shutdown():
        arm.send_goal(0.0, 0.0, 0.0, frame="ur_arm_starting_pose")
        rate.sleep()

#!/usr/bin/python3
import rospy
from arm_class import Arm
up_distance = 0.75

if __name__ == "__main__":
    rospy.init_node("arm_mapping_movement")
    arm = Arm(config_file_name="ur5e_info.yaml")
    while not rospy.is_shutdown():
        if rospy.has_param("/move_up"):
            if rospy.get_param("/move_up"):
                arm.send_goal(0.0, 0.0, up_distance)
                break


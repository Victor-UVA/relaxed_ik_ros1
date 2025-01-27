#!/usr/bin/python3

import rospy
from arm_class import Arm

x_stride = 0.025
y_stride = 0.05
z_stride = 0.4

FARTHER = 1
CLOSER = -1

if __name__ == "__main__":
    rospy.init_node("arm_mapping_movement")

    rate = rospy.Rate(50)
    x_direction = 0
    y_direction = 0
    z_direction = 0
    arm = Arm(config_file_name="ur5e_info.yaml")
    while not rospy.is_shutdown():
        # Move forward and back based on direction parameter
        arm_x = 0
        arm_y = 0
        arm_z = 0
        # if rospy.has_param("/x_direction"):
        #     x_direction = rospy.get_param("/x_direction")
        # if rospy.has_param("/y_direction"):
        #     y_direction = rospy.get_param("/y_direction")
        # if rospy.has_param("/z_direction"):
        #     y_direction = rospy.get_param("/z_direction")

        if rospy.has_param("/direction"):
            x_direction = rospy.get_param("direction")

        if x_direction == FARTHER:
            arm_x = -x_stride
        elif x_direction == CLOSER:
            arm_x = x_stride

        if rospy.has_param("/move_up"):
            if rospy.get_param("/move_up"):
                arm_z = z_stride
                rospy.set_param("/move_up", False)
        # if y_direction == FARTHER:
        #     arm_y = -y_stride
        # elif y_direction == CLOSER:
        #     arm_y = y_stride

        # if z_direction == FARTHER:
        #     arm_z = -z_stride
        # elif z_direction == CLOSER:
        #     arm_z = z_stride

        if arm_x or arm_y or arm_z:
            arm.send_goal(arm_x, arm_y, arm_z)
        rate.sleep()

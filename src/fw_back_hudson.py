#!/usr/bin/python3

from math import radians
import rospy
from arm_class import Arm

x_stride = 0.01
y_stride = 0.01
z_stride = 0.01

FARTHER = 1
CLOSER = -1

if __name__ == "__main__":
    rospy.init_node("arm_mapping_movement")

    rate = rospy.Rate(5)
    x_direction = 0
    y_direction = 0
    z_direction = 0
    arm = Arm()#home=[-2.53073, -1.570796327, -3.14159265359, 0.959931, 1.570796327, 0.0])
    while not rospy.is_shutdown():
        # Move forward and back based on direction parameter
        # break
        arm.send_goal(x_stride, 0.0, 0.0)
        rospy.sleep(5)
        arm.send_goal(0.0, y_stride, 0.0)
        rospy.sleep(5)
        arm.send_goal(0.0, 0.0, z_stride)
        rospy.sleep(5)
        break
        arm.send_goal(x_stride, 0.0, 0.0, frame="ur_arm_starting_pose")
        rospy.sleep(5)
        arm.send_goal(0.0, y_stride, 0.0, frame="ur_arm_starting_pose")
        rospy.sleep(5)
        arm.send_goal(0.0, 0.0, z_stride, frame="ur_arm_starting_pose")
        rospy.sleep(5)
        # arm.send_to_home()
        break
        arm_x = 0
        arm_y = 0
        arm_z = 0
        if rospy.has_param("/x_direction"):
            x_direction = rospy.get_param("/x_direction")
        if rospy.has_param("/y_direction"):
            y_direction = rospy.get_param("/y_direction")
        if rospy.has_param("/z_direction"):
            y_direction = rospy.get_param("/z_direction")

        if x_direction == FARTHER:
            arm_x = -x_stride
        elif x_direction == CLOSER:
            arm_x = x_stride

        if y_direction == FARTHER:
            arm_y = -y_stride
        elif y_direction == CLOSER:
            arm_y = y_stride

        if z_direction == FARTHER:
            arm_z = -z_stride
        elif z_direction == CLOSER:
            arm_z = z_stride

        if arm_x or arm_y or arm_z:
            arm.send_goal(arm_x, arm_y, arm_z)
        rate.sleep()

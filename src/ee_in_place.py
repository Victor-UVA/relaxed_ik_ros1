#!/usr/bin/python3

from tf_functions import pose_lookup
from geometry_msgs.msg import TransformStamped

import rospy
from arm_class import Arm

if __name__ == "__main__":
    rospy.init_node("ee_in_place")
    rate = rospy.Rate(50)
    arm = Arm(config_file_name="ur5e_info.yaml")
    t : TransformStamped = pose_lookup("map", "ur_arm_starting_pose")
    pos = t.transform.translation 
    rot = t.transform.rotation
    rospy.loginfo("Starting pose in map coordinates is ({}, {}, {})".format(pos.x, pos.y, pos.z))
    # arm.send_goal(0, 0, 0.2)
    # rospy.sleep(5)
    # while not rospy.is_shutdown():
    #     arm.send_goal(pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w, frame="map")
    #     rate.sleep()

    arm.send_joint_velocity(wrist3=0.1)
    rospy.sleep(5)

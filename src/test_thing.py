#!/usr/bin/python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray


rospy.init_node("test_thing")

# vel_pub = rospy.Publisher(
#             "/ur/ur_arm_vel_joint_traj_controller/command", JointTrajectory, queue_size=10)
# rate = rospy.Rate(5)
# while not rospy.is_shutdown():
#     msg = JointTrajectory()
#     point = JointTrajectoryPoint()
#     point.velocities = [0,
#                         0,
#                         5,
#                         0,
#                         0,
#                         0]
#     point.positions = [ -2.617993878, -0.5235987756, -1.5, 0.0, 1.570796327, 0 ]
#     # point.accelerations = [0,0,0,0,0,0]
#     # point.effort = []
#     msg.points.append(point)
#     point = JointTrajectoryPoint()
#     point.velocities = [0,
#                         0,
#                         5,
#                         0,
#                         0,
#                         0]
#     point.positions = [ -2.617993878, -0.5235987756, 0, 0.0, 1.570796327, 0 ]
#     # point.accelerations = [0,0,0,0,0,0]
#     # point.effort = []
#     msg.points.append(point)
#     msg.joint_names = ['ur_arm_elbow_joint',
#                         'ur_arm_shoulder_lift_joint',
#                         'ur_arm_shoulder_pan_joint',
#                         'ur_arm_wrist_1_joint',
#                         'ur_arm_wrist_2_joint',
#                         'ur_arm_wrist_3_joint']

#     msg.header.stamp = rospy.Time.now()
#     point.time_from_start = rospy.Duration(secs=10)


#     vel_pub.publish(msg)
#     rate.sleep()

vel_pub = rospy.Publisher(
            "/ur/ur_arm_joint_group_vel_controller/command", Float64MultiArray, queue_size=10)

rate = rospy.Rate(5)
while not rospy.is_shutdown():

    msg = Float64MultiArray()
    msg.data=[0,0,0,0,0,0.5]
    vel_pub.publish(msg)
    rate.sleep()
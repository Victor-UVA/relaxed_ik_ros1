#!/usr/bin/python3

import readchar
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose, Twist
from std_msgs.msg import Bool
from relaxed_ik_ros1.msg import EEPoseGoals
import transformations as T
import rosbag

rospy.init_node('rosbag_drive')

cmd_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=5)
bag_num = 0

topic_name = '/joy_teleop/cmd_vel'

rate = rospy.Rate(30)
print('Waiting')
rospy.sleep(5)
print('Driving')
while not rospy.is_shutdown():

    # print(bag_num)
    
    if bag_num == 0:
        bag_lower = rosbag.Bag('machine_lower.bag')
        # print(bag_lower)

        for topic, msg, t in bag_lower.read_messages(topics={topic_name}):
            # print(topic)
            # print(t)
            # print(msg)
            cmd_pub.publish(msg)
            rate.sleep()

        bag_lower.close()

        bag_num = bag_num+1
    elif bag_num == 1:
        bag_turn = rosbag.Bag('machine_turn.bag')
        
        for topic, msg, t in bag_turn.read_messages(topics={topic_name}):
            # print(msg)
            cmd_pub.publish(msg)
            rate.sleep()

        bag_turn.close()
        bag_num = bag_num+1
    elif bag_num == 2:
        rospy.set_param('/move_up',True)

        rospy.sleep(10)
        bag_num = bag_num+1
    elif bag_num == 3:
        bag_upper = rosbag.Bag('machine_upper.bag')
        
        for topic, msg, t in bag_upper.read_messages(topics={topic_name}):
            # print(msg)
            cmd_pub.publish(msg)
            rate.sleep()

        bag_upper.close()
        bag_num = bag_num+1
    else:
        final_msg = Twist()
        # print(final_msg)
        cmd_pub.publish(final_msg)
        rate.sleep()



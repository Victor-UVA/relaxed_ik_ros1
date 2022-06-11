#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Joy

def joyCB(data):
    if data.buttons[1] == 1:
        rospy.set_param('/move_up',True)
    elif data.buttons[2] == 1:
        rospy.set_param('/move_up',True)

def main():
    rospy.init_node('joy_move')

    rospy.Subscriber("/joy_teleop/joy", Joy, joyCB) ###########################################
    
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
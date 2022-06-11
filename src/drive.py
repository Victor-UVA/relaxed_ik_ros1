#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    freq = 10
    speed = 0.1
    distance = 1.75
    rate = rospy.Rate(freq) # 10hz
    i = 0
    rospy.sleep(5)
    while not rospy.is_shutdown():
        if i<(distance/speed)*freq:
            direction = 1
        elif i>(distance/speed)*2*freq:
            direction = 0
        elif i==(distance/speed)*freq:
            rospy.set_param("/move_up",True)
            rospy.sleep(5)
        else:
            direction = -1
        msg = Twist()

        msg.linear.x = direction*speed
        msg.angular.z = 0.0
        rospy.loginfo(msg)
        rospy.loginfo(i)
        pub.publish(msg)
        
        i = i+1
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
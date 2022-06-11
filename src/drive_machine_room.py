#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
import matplotlib.pyplot as plt
import time

def talker():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    
    freq = 10
    speed = 0.1
    rate = rospy.Rate(freq) # 10hz

    lin_s = 0.1
    ang_s = 0.1

    x = list()
    y = list()
    theta = list()

    x.append(0)
    y.append(0)
    theta.append(0)

    linear_speed = [lin_s, 0, lin_s, 0, lin_s, 0, lin_s, 0, lin_s, 0, lin_s, 0, lin_s, 0, lin_s, -lin_s, 0, -lin_s, 0, -lin_s, 0, lin_s, 0, lin_s, 0, lin_s, 0, lin_s, 0, lin_s, 0, lin_s, 0, lin_s, 0, lin_s, 0]
    angular_speed = [0, ang_s, 0, -ang_s, 0, -ang_s, 0, -ang_s, 0, ang_s, 0, -ang_s, 0, ang_s, 0, 0, -ang_s, 0, ang_s, 0, ang_s, 0, ang_s, 0, ang_s, 0, ang_s, 0, -ang_s, 0, -ang_s, 0, ang_s, 0, -ang_s, 0, 0]
    steps = [44, 201, 235, 314, 351, 430, 506, 550, 592, 636, 662, 723, 783, 844, 945, 1046, 1107, 1167, 1228, 1232, 1389, 1490, 1507, 1542, 1682, 1738, 1895, 1970, 2040, 2081, 2168, 2241, 2398, 2470, 2627, 2671, 10000]

    i = 0
    step = 0

    figure, ax = plt.subplots(figsize=(4,5))
    plt.ion()
    plot1, = ax.plot(x, y)

    rospy.sleep(5)
    while not rospy.is_shutdown():
        # move along wall for specified distance
        # if at keypoint adjust the arm to new position
        # if i<(distance/speed)*freq:
        #     direction = 1
        # elif i>(distance/speed)*2*freq:
        #     direction = 0
        # elif i==(distance/speed)*freq:
        #     rospy.set_param("/move_up",True)
        #     rospy.sleep(5)
        # else:
        #     direction = -1
        
        msg = Twist()

        if i<steps[step]:
            msg.linear.x = linear_speed[step]
            msg.angular.z = angular_speed[step]
        elif i==steps[step]:
            msg.linear.x = linear_speed[step]
            msg.angular.z = angular_speed[step]
            step = step + 1
        else:
            msg.linear.x = 0
            msg.angular.z = 0
        
        x.append(x[-1] + math.cos(theta[-1])*msg.linear.x/10)
        y.append(y[-1] + math.sin(theta[-1])*msg.linear.x/10)
        theta.append(theta[-1] + msg.angular.z/10)
        # print("( " + str(x) + ", " + str(y) + ", " + str(theta) + " )")
        # plt.scatter(y,x)
        # plt.pause(0.005)
        # plt.axis('equal')
        # plot1.set_xdata(x)
        # plot1.set_ydata(y)
        
        # figure.canvas.draw()
        # figure.canvas.flush_events()
        # # time.sleep(0.1)
        # plt.show()

        # msg = Twist()

        # msg.linear.x = direction*speed
        # msg.angular.z = 0.0
        # rospy.loginfo(msg)
        # rospy.loginfo(i)
        pub.publish(msg)
        
        i = i+1
        # print(i)

        if i>steps[35]:
            plt.scatter(y,x)
            plt.pause(0.005)
            plt.axis('equal')
            # print(msg)
            i = steps[-1]+1
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
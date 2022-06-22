#!/usr/bin/python3

#### ASSUMES ARM FACING FRONT OF HUSKY AND STARTING FROM BOT LEFT CORNER ####
from sympy import capture, true
import rospy
from arm_class import Arm
from std_msgs.msg import Bool, Float64
import threading
import multiprocessing
import pyflycap2

# Tunable constants
x_stride = 0.05  # m
y_stride = 0.05  # m
z_stride = 0.05  # m
desired_distance = 0.15  # m
tolerance = 0.001  # m
fovZ = 9.15  # cm
fovY = 15.5  # cm
scanHeight = 20  # ?
scanWidth = 50  # ?
heightSteps = scanHeight / (fovY - 1)
widthSteps = scanWidth / (fovZ - 1)

# Flags
correct_distance = False

# Initialize global arm
# may need to go in main, not exactly sure how python global works
arm = Arm(config_file_name="ur5e_info.yaml")


def prox_cb(data: Float64):
    global correct_distance, arm
    distance_m = data.data/100  # converting from centimeters to meters
    if distance_m > desired_distance-tolerance:
        arm.send_goal(0.0, y_stride, 0.0)
    elif distance_m < desired_distance+tolerance:
        arm.send_goal(0.0, -y_stride, 0.0)
    else:
        correct_distance = True
        return
    correct_distance = False


def capture_pic():
    take_pics_pub = rospy.Publisher('/rps/take_pics', Bool, queue_size=1)
    take_pics_pub.publish(Bool(True))
    rospy.wait_for_message("/rps/done", Bool)
    take_pics_pub.publish(Bool(False))
    # save pictures


def move_sequence():
    global arm
    for i in range(widthSteps/2):  # traverse width
        capture_pic()
        for j in range(heightSteps):  # move up
            arm.send_goal(0.0, 0.0, z_stride)
            rospy.sleep(1)
            capture_pic()
        arm.send_goal(x_stride, 0.0, 0.0)  # move to the right
        capture_pic()
        for k in range(heightSteps):  # move down
            arm.send_goal(0.0, 0.0, -z_stride)
            rospy.sleep(1)
            capture_pic()
        arm.send_goal(x_stride, 0.0, 0.0)  # move to the right
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("rps_control")
    arm_distance_sub = rospy.Subscriber('/rps/arm_distance', Float64, prox_cb)
    rate = rospy.Rate(5)
    move_thread = threading.Thread(target=move_sequence)
    lock = threading.Lock()
    cond = threading.Condition()
    cond.wait_for()
    while not rospy.is_shutdown():
        if not correct_distance:
            move_thread.start()
            break
        rate.sleep()

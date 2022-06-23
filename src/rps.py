#!/usr/bin/python3

#### ASSUMES ARM FACING FRONT OF HUSKY AND STARTING FROM BOT LEFT CORNER ####
import rospy
from arm_class import Arm
from std_msgs.msg import Bool, Float64
from pyflycap2.interface import Camera


class RPS:
    def __init__(self, num_pics: int, file_prefix: str, arm_config_file_name: str):
        # Tunable constants
        self.x_stride = 0.05  # m
        self.y_stride = 0.05  # m
        self.z_stride = 0.05  # m
        self.desired_distance = 0.15  # m
        self.tolerance = 0.001  # m
        self.fovZ = 9.15  # cm
        self.fovY = 15.5  # cm
        self.scanHeight = 20  # ?
        self.scanWidth = 50  # ?
        self.heightSteps = self.scanHeight / (self.fovY - 1)
        self.widthSteps = self.scanWidth / (self.fovZ - 1)
        self.file_prefix = file_prefix
        self.num_pics = num_pics
        self.current_pic_num = 0
        # Flags
        self.correct_distance = False
        # Initialize arm
        self.arm = Arm(config_file_name=arm_config_file_name)
        # Topic stuff
        self.arm_distance_sub = rospy.Subscriber(
            '/rps/arm_distance', Float64, self.prox_cb)
        self.take_pics_pub = rospy.Publisher(
            '/rps/take_pics', Bool, queue_size=1)
        # Initialize camera
        self.cam = Camera()
        self.cam.connect()
        self.cam.set_drop_mode(drop=False)  # allow buffering

    def prox_cb(self, data: Float64):
        distance_m = data.data/100  # converting from centimeters to meters
        if distance_m > self.desired_distance-self.tolerance:
            self.arm.send_goal(0.0, self.y_stride, 0.0)
        elif distance_m < self.desired_distance+self.tolerance:
            self.arm.send_goal(0.0, -self.y_stride, 0.0)
        else:
            self.correct_distance = True
            return
        self.correct_distance = False

    def capture_pic(self):
        self.take_pics_pub.publish(Bool(True))
        rospy.wait_for_message("/rps/done", Bool)
        self.take_pics_pub.publish(Bool(False))
        # save pictures
        for i in range(self.num_pics):
            self.cam.read_next_image()
            self.cam.save_current_image(
                self.file_prefix + "-" + str(self.current_pic_num))
            self.current_pic_num += 1

    def move_sequence(self):
        for i in range(self.widthSteps/2):  # traverse width
            self.capture_pic()
            for j in range(self.heightSteps):  # move up
                self.arm.send_goal(0.0, 0.0, self.z_stride)
                rospy.sleep(1)
                self.capture_pic()
            self.arm.send_goal(self.x_stride, 0.0, 0.0)  # move to the right
            self.capture_pic()
            for k in range(self.heightSteps):  # move down
                self.arm.send_goal(0.0, 0.0, -self.z_stride)
                rospy.sleep(1)
                self.capture_pic()
            self.arm.send_goal(self.x_stride, 0.0, 0.0)  # move to the right
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("rps_control")
    rate = rospy.Rate(5)
    rps = RPS(file_prefix="test", arm_config_file_name="ur5e_info.yaml")
    while not rospy.is_shutdown():
        if rps.correct_distance:
            rps.move_sequence()
            break
        rate.sleep()

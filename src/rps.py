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
        self.y_stride = 0.003  # m
        self.z_stride = 0.05 # m
        self.desired_distance = 0.17  # m
        self.actual_distance = self.desired_distance
        self.tolerance = 0.01  # m
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
        self.cam = Camera(index="")
        self.cam.connect()
        self.cam.set_drop_mode(drop=False)  # allow buffering

    def prox_cb(self, data: Float64):
        self.actual_distance = data.data/100  # converting from centimeters to meters


    def calibrate_distance(self):
        self.arm.send_goal(0.0, self.actual_distance-self.desired_distance, 0.0)
        
        # while self.actual_distance < self.desired_distance - self.tolerance or self.actual_distance > self.desired_distance + self.tolerance:
        #     self.arm.send_goal(0.0, self.actual_distance-self.desired_distance, 0.0)
        #     rospy.wait_for_message("/rps/arm_distance", Float64)
    
    def capture_pic(self):
        self.take_pics_pub.publish(Bool(True))
        rospy.loginfo("Taking pictures")
        # rospy.wait_for_message("/rps/done", Bool)
        self.take_pics_pub.publish(Bool(False))
        rospy.loginfo("Done taking pictures")
        # save pictures
        for i in range(self.num_pics):
            self.cam.read_next_image()
            self.cam.save_current_image(
                self.file_prefix + "-" + str(self.current_pic_num))
            self.current_pic_num += 1

    def move_sequence(self):
        for i in range(round(self.widthSteps/2)):  # traverse width
            self.capture_pic()
            for j in range(round(self.heightSteps)):  # move up
                self.arm.send_goal(0.0, 0.0, self.z_stride)
                rospy.sleep(1)
                self.capture_pic()
            self.arm.send_goal(self.x_stride, 0.0, 0.0)  # move to the right
            self.capture_pic()
            for k in range(round(self.heightSteps)):  # move down
                self.arm.send_goal(0.0, 0.0, -self.z_stride)
                rospy.sleep(1)
                self.capture_pic()
            self.arm.send_goal(self.x_stride, 0.0, 0.0)  # move to the right
            rospy.sleep(1)


if __name__ == "__main__":
    rospy.init_node("rps_control")
    rate = rospy.Rate(2)
    rps = RPS(num_pics=8, file_prefix="test", arm_config_file_name="ur5e_info.yaml")
    while not rospy.is_shutdown():
        # rps.calibrate_distance()
        rps.move_sequence()
        
        rate.sleep()

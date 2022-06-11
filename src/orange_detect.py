#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from math import pi
import roslib
#roslib.load_manifest('my_package')
import cv2
import rospy
import sys
import imutils
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import pdb
import tf

orange_lower = (0,155,100)
orange_upper = (10,255,255)
bridge = CvBridge()

# @dataclass
# class PixelLoc:
#     x: int
#     y: int

# @dataclass
# class XYZLoc:
#     x: float
#     y: float
#     z: float

class Tracker3D():
    rgbFOV = [69,42]
    depthFOV = [74,62]
    pixToDegree = (np.pi/180)*float(86)/640
    center_pixel = (320,240)
    lamb = np.linalg.norm(np.array([0.02,1,-0.003]))
    min_error = 0.1

    def __init__(self,img_topic_name="/camera/color/image_raw",depth_topic_name="/camera/aligned_depth_to_color/image_raw",see_image=False,camerainfo_topic_name="/camera/depth/camera_info"):
        
        self.image_sub = rospy.Subscriber(img_topic_name,Image,self.image_cb)
        self.depth_sub = rospy.Subscriber(depth_topic_name,Image,self.depth_cb)
        self.camerainfo_sub = rospy.Subscriber(camerainfo_topic_name,CameraInfo,self.camerainfo_cb)
        # self.depth_cam_info = rospy.Subscriber(depth_camera_topic,self.depth_cam_cb)
        self.pos_pub = rospy.Publisher('ballxyz',PoseWithCovariance,queue_size=10)
        self.viz_pub = rospy.Publisher('ball_maker',Marker,queue_size=10)
        self.ballloc_pixel = [0,0]
        self.ballloc_xyz = [0,0,0]
        self.pixelvec = None
        self.worldvec = None
        self.learning_rate = 0.2 # = 1.0 => no averaging
        self.cv_image = None
        self.depth_image = None
        self.K = []
        self.Rt = np.array([[1,0,0],[0,0,1],[0,-1,-0]])
        self.mask = None
        self.d = 0.0
        self.thetax = 0.0
        self.thetay = 0.0
        self.phi = 0.0
        self.error = self.min_error

        self.listener = tf.TransformListener()
        
        # plt.ion()
        # self.fig = plt.figure()
        # # ax = self.fig.add_subplot(111, projection='3d')
        # ax = Axes3D(self.fig)
        # print(type(ax))
        # self.plt_xyz = ax.scatter(self.ballloc_xyz[0],self.ballloc_xyz[1],self.ballloc_xyz[2])
        # plt.show()
        # pdb.set_trace()

        # Wait for messages to be published on image and depth topics
        print("Waiting for image and depth topic")
        rospy.wait_for_message(img_topic_name,Image)
        rospy.wait_for_message(depth_topic_name,Image)
        print("-----> Messages received")

        self.rate = rospy.Rate(20)

    def get_depth(self):
        print(self.cv_image.shape)
        print(self.depth_image.shape)
        xoffsets = [-1,0,1]
        yoffsets = [-1,0,1]

        avg_depth = 0.0
        for xoffset in xoffsets:
            for yoffset in yoffsets:
                xcenter = max(min(self.ballloc_pixel[0]+xoffset,639),0)
                ycenter = max(min(self.ballloc_pixel[1]+yoffset,479),0)
                avg_depth += 0.001*self.depth_image[ycenter][xcenter]
        avg_depth /= len(xoffsets)*len(yoffsets)

        # xdepth = int(self.rgbFOV[0]*float(self.ballloc_pixel[0]-self.center_pixel[0])/self.depthFOV[0]) + int(640/2)
        # ydepth = int(self.rgbFOV[1]*float(self.ballloc_pixel[1]-self.center_pixel[1])/self.depthFOV[1]) + int(480/2)
        # xdepth = max(min(xdepth,639),0)
        # ydepth = max(min(ydepth,479),0)
        print(self.ballloc_pixel)
        xdepth = self.ballloc_pixel[0]
        ydepth = self.ballloc_pixel[1]
        self.pixelvec = np.array([[xdepth],[ydepth],[1]])

        # xdepth = min(int(86*float(self.ballloc_pixel[0])/65),639)
        # ydepth = min(int(86*float(self.ballloc_pixel[1])/65),479)
        print("(xdepth,ydepth): ({},{})".format(xdepth,ydepth))
        # self.d = 0.001*float(self.depth_image[ydepth][xdepth])
        self.d = avg_depth

    def get_xyz(self):
        self.get_depth()
        print("depth: {}".format(self.d))
        if self.d < 0.05 or self.d > 20.0:
            return

        self.worldvec = (self.d/self.lamb)*np.matmul(np.matmul(self.Rt,np.linalg.inv(self.K)),self.pixelvec)
        # print("world vec:")
        # print(self.worldvec)

       
        # Wihtout K matrix
        # print(self.ballloc_pixel)
        # self.theta = self.pixToDegree*float(self.ballloc_pixel[0]-self.center_pixel[0])
        # self.theta = self.learning_rate*self.theta + (1-self.learning_rate)*self.theta
        # print("theta: {}".format(self.theta))
        # x = self.d*np.tan(self.theta) #self.d*np.sin(self.theta)
        # y = self.d
        y = -self.worldvec[0]
        x = self.worldvec[1]
        # z = self.worldvec[2]
        z = 0.0

        # Set x,y coord
        self.ballloc_xyz[0] = self.learning_rate*x + (1-self.learning_rate)*self.ballloc_xyz[0]
        self.ballloc_xyz[1] = self.learning_rate*y + (1-self.learning_rate)*self.ballloc_xyz[1]
        self.ballloc_xyz[2] = self.learning_rate*z + (1-self.learning_rate)*self.ballloc_xyz[2]

        self.error = max(self.min_error,(self.d-0.3)*self.min_error)

    def pub_xy(self):
        msg = PoseWithCovariance()
        msg.pose.position.x = self.ballloc_xyz[0]
        msg.pose.position.y = self.ballloc_xyz[1]
        msg.pose.position.z = self.ballloc_xyz[2]
        msg.covariance[0] = self.error
        msg.covariance[8] = self.error
        msg.covariance[15] = self.error
        # msg.covariance[15] = 0.0
        self.pos_pub.publish(msg)

    def pub_viz(self):
        msg = Marker()
        msg.type = msg.SPHERE
        msg.color.r = 1.0
        msg.color.a = 1.0
        msg.scale.x = self.error
        msg.scale.y = self.error
        msg.scale.z = self.error
        msg.header.frame_id = "/map"
        msg.pose.position.x = self.ballloc_xyz[0]
        msg.pose.position.y = self.ballloc_xyz[1]
        msg.pose.position.z = self.ballloc_xyz[2]
        self.viz_pub.publish(msg)

    def viz_3d(self):
        self.plt_xyz.set_offsets(self.ballloc_xyz)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def image_cb(self,data):
        try:
		    self.cv_image = bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image_hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV)
	    #(rows,cols,channels) = cv_image.shape
        # cv2.circle(self.cv_image,(320,240),10,(255,0,0))

	    #blurred = cv2.GaussianBlur(cv_image,(11,11),0)
        mask = cv2.inRange(cv_image_hsv,orange_lower,orange_upper)
        mask = cv2.erode(mask,None,iterations=2)
        mask = cv2.dilate(mask,None,iterations=2)
        self.mask = mask

	    # find contours in the mask and initialize the current
	    # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	    	cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(self.cv_image, cnts[0], -1, (0,0,255), 2)
        cnts = imutils.grab_contours(cnts)
        
        center = None
	    #print (cnts)
	    # only proceed if at least one contour was found
        # print(cnts)
        if len(cnts)>0:
            c = max(cnts, key=cv2.contourArea)
            # c = cnts
            # print(c)
            ((x,y),radius) = cv2.minEnclosingCircle(c)
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(self.cv_image,[box], 0 , (0,0,255),2)
            self.ballloc_pixel = [int(x),int(y)]
            
            if radius > 3:
                cv2.circle(self.cv_image, (int(x),int(y)), int(radius),(0,0,255),2)

        

    def depth_cb(self,data):
        try:
            self.depth_image = bridge.imgmsg_to_cv2(data,"16UC1")
        except CvBridgeError as e:
            print(e)

    def camerainfo_cb(self,data):
        # print("Camera info:")
        # print(type(data.K))
        # print(np.reshape(np.array(data.K),[3,3]))
        self.K = np.reshape(np.array(data.K),[3,3])
        # print(np.linalg.inv(self.K))

    # def depth_cam_cb(self,data):
    #     print(type(data))

if __name__ == "__main__":
    rospy.init_node("measure_3d")
    tracker = Tracker3D()
    viz_img = True
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        tracker.get_xyz()
        tracker.pub_xy()
        tracker.pub_viz()
        print("Ball location: ({},{})".format(tracker.ballloc_xyz[0],tracker.ballloc_xyz[1]))
        if viz_img:
            # tracker.viz_3d()
            cv2.circle(tracker.cv_image, (int(tracker.center_pixel[0]),int(tracker.center_pixel[1])), int(10),(255,0,0),2)
            cv2.imshow("Image window",tracker.cv_image)
            # cv2.imshow("Image window",tracker.mask)
            cv2.waitKey(1) & 0xFF
        rate.sleep()
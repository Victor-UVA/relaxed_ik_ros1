#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from numpy.core.defchararray import array
from numpy.core.fromnumeric import shape, sort
from numpy.core.numeric import ones
from numpy.lib.function_base import append
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
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError
import pdb
import tf
import tf2_ros
import tf_conversions
from PIL import Image as im


orange_lower = (5,155,100)
orange_upper = (15,255,255)
red_lower = (1,230,100)
red_upper = (5,255,255)
bridge = CvBridge()

class Tracker3D():
    rgbFOV = [69,42]
    depthFOV = [74,62]
    
    center_pixel = (320,240)
    lamb = (1)/np.linalg.norm(np.array([0.02,1.0,-0.1]))
    min_error = 0.1

    def __init__(self,img_topic_name="/camera/color/image_raw",depth_topic_name="/camera/aligned_depth_to_color/image_raw",see_image=False,camerainfo_topic_name="/camera/depth/camera_info"):
        
        self.image_sub = rospy.Subscriber(img_topic_name,Image,self.image_cb)
        self.depth_sub = rospy.Subscriber(depth_topic_name,Image,self.depth_cb)
        self.camerainfo_sub = rospy.Subscriber(camerainfo_topic_name,CameraInfo,self.camerainfo_cb)
        self.pos_pub = rospy.Publisher('Panel_center_xyz',PoseWithCovariance,queue_size=10)
        self.viz_pub = rospy.Publisher('panel_maker',Marker,queue_size=10)

        self.panel_loc_xyz = np.zeros([3,])
        self.cv_image = None
        self.depth_image = None
        self.masked_image = None
        self.masked_image_neg = None
        self.K = []
        self.Rt = np.array([[0,0,1],[0,-1,0],[-1,0,0],])
        self.mask = None
        self.d = 0.0
        self.panel_pixel = None
        self.switches_pixles = None
        self.switch_xyz = None

        self.listener = tf.TransformListener()

        self.panel_corners_pixel = [[0,0],[0,0],[0,0],[0,0]] 
        self.pixelvec = None
        self.learning_rate = 0.2 # = 1.0 => no averaging       
        self.cropped_image = None
        self.di = np.array([1,1,1,1]) # Depth of each point
        self.error = self.min_error

    

    def depth_cb(self,data):
        try:
            self.depth_image = bridge.imgmsg_to_cv2(data,"16UC1")
        except CvBridgeError as e:
            print(e)

    def camerainfo_cb(self,data):
        # print("Camera info:")
        self.K = np.reshape(np.array(data.K),[3,3])

    def image_cb(self,data):
        try:
		    self.cv_image = bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)
        print(np.shape(self.cv_image))
        cv_image_hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV)
	    
        mask = cv2.inRange(cv_image_hsv,orange_lower,orange_upper)
        mask = cv2.erode(mask,None,iterations=2)
        mask = cv2.dilate(mask,None,iterations=2)
        self.mask = mask
        

	    # find contours in the mask and initialize the current
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	    	cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        if len(cnts)>0:
            c = max(cnts, key=cv2.contourArea)
            ((x,y),radius) = cv2.minEnclosingCircle(c)
            self.panel_pixel = [int(x),int(y)]
            
            if radius > 3:
                cv2.circle(self.cv_image, (int(x),int(y)), int(radius),(0,0,255),2)





if __name__ == "__main__":
    rospy.init_node("panel_3d")
    tracker = Tracker3D()
    # tracker.camera_transform()
    viz_img = True
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        # tracker.get_panel_xyz()
        # tracker.pub_panel_xyz()
        # tracker.pub_viz()
        # tracker.get_switch_pixel()
        # tracker.get_switches_xyz()
        # tracker.show_depth_map()
        # print("Ball location: ({},{})".format(tracker.ballloc_xyz[0],tracker.ballloc_xyz[1]))
        if viz_img:
            # tracker.viz_3d()
            # cv2.circle(tracker.cv_image, (int(tracker.center_pixel[0]),int(tracker.center_pixel[1])), int(10),(255,0,0),2)
            cv2.imshow("Image window 1",tracker.cv_image)
            # cv2.imshow("Image window 2",tracker.masked_image)
            # cv2.imshow("Image window 3",tracker.masked_image_negdd)
            # cv2.imshow("Image window 3",tracker.cropped_image)
            # cv2.imshow("Image window",tracker.depth_image)
            cv2.waitKey(1) & 0xFF
        rate.sleep()

    

    
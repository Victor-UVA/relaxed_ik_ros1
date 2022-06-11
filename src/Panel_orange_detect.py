#!/usr/bin/env python3
from operator import pos
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
from geometry_msgs.msg import PoseWithCovariance, PoseArray, Pose
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError
import pdb
import tf
import tf2_ros
import tf_conversions
from PIL import Image as im
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


#tf_prefix = rospy.get_param("/UR_Dingo/ur_hardware_interface/tf_prefix")

orange_lower = (5,155,100)
orange_upper = (15,255,255)
red_lower = (1,200,100)
red_upper = (5,255,255)
yellow_lower = (35,70,100)
yellow_upper = (45,120,200) 
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
    
    center_pixel = (360,640)
    lamb = 0.94/np.linalg.norm(np.array([0.002,0.925,-0.003]))
    min_error = 0.1

    def __init__(self,img_topic_name="/camera/color/image_raw",depth_topic_name="/camera/aligned_depth_to_color/image_raw",see_image=False,camerainfo_topic_name="/camera/depth/camera_info"):
        
        self.image_sub = rospy.Subscriber(img_topic_name,Image,self.image_cb)
        self.depth_sub = rospy.Subscriber(depth_topic_name,Image,self.depth_cb)
        self.camerainfo_sub = rospy.Subscriber(camerainfo_topic_name,CameraInfo,self.camerainfo_cb)
        self.pointcloud_sub = rospy.Subscriber('/camera/depth_registered/points',PointCloud2,self.pc2Sub)
        self.processing = False
        self.currentCloud = None
        # self.depth_cam_info = rospy.Subscriber(depth_camera_topic,self.depth_cam_cb)
        self.pos_pub = rospy.Publisher('Panel_center_xyz',PoseWithCovariance,queue_size=10)
        self.switch_pos_pub = rospy.Publisher('Switch_xyz',PoseArray,queue_size=10)
        self.viz_pub = rospy.Publisher('panel_maker',Marker,queue_size=10)
        self.panel_corners_pixel = [[0,0],[0,0],[0,0],[0,0]]
        self.panel_loc_xyz = np.zeros([3,])
        self.pixelvec = None
        self.switch_xyz = None
        self.learning_rate = 0.2 # = 1.0 => no averaging
        self.cv_image = None
        self.depth_image = None
        self.masked_image = None
        self.masked_image_neg = None
        self.cropped_image = None
        self.switches_pixles = None
        self.K = []
        self.Rt = np.array([[0,0,-1],[-1,0,0],[0,-1,0]])
        self.mask = None
        self.d = 0.0
        self.di = np.array([1,1,1,1]) # Depth of each point
        self.thetax = 0.0
        self.thetay = 0.0
        self.phi = 0.0
        self.error = self.min_error
        self.num_switches_detected = 0
        

        self.switch_flag = False

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

    def camera_transform(self):
        br = tf2_ros.StaticTransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        #t.header.frame_id = tf_prefix + "shoulder_link"
        t.header.frame_id = "shoulder_link"
        t.child_frame_id = "camera_link"

        t.transform.translation.x = float(-0.026)
        t.transform.translation.y = float(0.0)
        t.transform.translation.z = float(0.167)
        
        q = tf_conversions.transformations.quaternion_from_euler(0,0,3.14)

        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])

        br.sendTransform(t)

    def panel_pc(self,pixel):
        if self.currentCloud is not None:
            
            try:
                xPixel = pixel[0]
                yPixel = pixel[1]
                # print(pixel)
                self.processing = True

                gen = pc2.read_points(self.currentCloud, skip_nans=True, field_names=('x','y','z'), uvs=[[int(xPixel),int(yPixel)]])
                # print(gen)

                for p in gen:
                    x = p[0]
                    y = p[1]
                    z = p[2]
                    break
                self.processing = False
                # print('point cloud',x,y,z)
                return [x,y,z]
            except:
                return [0,0,0]
    
    def pc2Sub(self,data):
        # self.currentCloud
        if not self.processing:
            self.currentCloud = data

    def get_panel_location(self):
        print(self.cv_image.shape)
        xwindow = [-1,0,1]
        ywindow = [-1,0,1]
        avg_depth = 0.0
        
        for xi in xwindow:
            for yi in ywindow:
                xcenter = max(min(self.panel_center_pixel[0]+xi,639),0)
                ycenter = max(min(self.panel_center_pixel[1]+yi,479),0)
                avg_depth += 0.001*self.depth_image[ycenter][xcenter]
        avg_depth /= len(xwindow)*len(ywindow)
        

        # xboundleft = max(self.panel_corners_pixel[0][0],self.panel_corners_pixel[0][1])   - 10
        # xboundright = min(self.panel_corners_pixel[0][2],self.panel_corners_pixel[0][3])  + 10
        # yboundtop = min(self.panel_corners_pixel[1][0],self.panel_corners_pixel[1][3])    + 10 
        # yboundbottom = max(self.panel_corners_pixel[1][1],self.panel_corners_pixel[1][2]) - 10        
        
        # d = np.zeros([4,1])
        
        # d[0] = 0.001*self.depth_image[yboundbottom][xboundright]
        # d[1] = 0.001*self.depth_image[yboundtop][xboundleft]
        # d[2] = 0.001*self.depth_image[yboundbottom][xboundleft]
        # d[3] = 0.001*self.depth_image[yboundtop][xboundright]
        
        # nonzero = np.sum(d>0)
        # avg_depth = np.sum(d)/nonzero
        # self.di = d
        # print(np.shape(self.depth_image))s
        # cv2.rectangle(self.cv_image,(xboundright,yboundbottom),(xboundleft,yboundtop),(255,255,255),4)
        
        xdepths = self.panel_center_pixel[0]
        ydepths = self.panel_center_pixel[1]
        self.pixelvec = np.array([xdepths,ydepths,1])
        
        self.d = avg_depth
        
        
        # ll = len(self.biggest_contour)-1
        # print(len(self.biggest_contour),'++++++++++++')
        # contour_depth = 0
        # for i in range(ll):
        #     contour_depth += self.depth_image[self.biggest_contour[i][0][1]][self.biggest_contour[i][0][0]]
        # xdepths = self.biggest_contour[:,0,0]
        # ydepths = self.biggest_contour[:,0,1]
        # zdepths = np.ones(len(xdepths))
        # self.pixelvec =  np.array([xdepths,ydepths,list(zdepths)])
        # print(0.001*contour_depth/ll,'Average depth')
                

    def get_panel_xyz(self):
        self.get_panel_location()
        # print("Depth of panel: {}".format(self.d))
        # print(self.pixelvec)
        if self.d < 0.05 or self.d > 20.0:
            return
        pixelvec = self.pixelvec
        # print(np.transpose(pixelvec))
        self.panel_loc_xyz = (self.d/self.lamb)*np.matmul(np.matmul(self.Rt,np.linalg.inv(self.K)),pixelvec)
        
             
        
    def pub_panel_xyz(self):
        msg = PoseWithCovariance()
        # msg.pose.position.x = np.mean(self.panel_loc_xyz[0][:])
        # msg.pose.position.y = -np.mean(self.panel_loc_xyz[1][:])
        # msg.pose.position.z = np.mean(self.panel_loc_xyz[2][:])
        xyz = self.panel_pc(self.panel_center_pixel)
        # print('XYZ',xyz)
        if xyz is not None:
            self.panel_loc_xyz[0] = xyz[0]
            self.panel_loc_xyz[1] = xyz[1]
            self.panel_loc_xyz[2] = xyz[2]
            msg.pose.position.x = -self.panel_loc_xyz[2]
            msg.pose.position.y = self.panel_loc_xyz[0]
            msg.pose.position.z = -self.panel_loc_xyz[1]
            msg.covariance[0] = self.error
            msg.covariance[8] = self.error
            msg.covariance[15] = self.error
            # # msg.covariance[15] = 0.0
            self.pos_pub.publish(msg)
            br = tf.TransformBroadcaster()
            br.sendTransform((xyz[2],-xyz[0],-xyz[1]),(0.0,0.0,0.0,1.0),rospy.Time.now(),"Panel_Center","camera_link")

    def pub_viz(self):
        
        msg = Marker()
        msg.type = msg.CUBE
        msg.color.b = 1.0
        msg.color.a = 0.5
        msg.scale.x = 0.05
        msg.scale.y = 0.1
        msg.scale.z = 0.1
        msg.header.frame_id = "/camera_link"
        print('Panel',np.mean(self.panel_loc_xyz[0]),np.mean(self.panel_loc_xyz[1]),np.mean(self.panel_loc_xyz[2]))
        msg.pose.position.x = -np.mean(self.panel_loc_xyz[0])
        msg.pose.position.y = -np.mean(self.panel_loc_xyz[1])
        msg.pose.position.z = np.mean(self.panel_loc_xyz[2])
         
        self.viz_pub.publish(msg)

    def viz_3d(self):
        self.plt_xyz.set_offsets(self.switch_xyz)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def get_switch_pixel(self):
        pos_idx = np.transpose(np.where(self.mask==255))
        cv_image_hsv = cv2.cvtColor(self.masked_image_neg,cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(cv_image_hsv,red_lower,red_upper)
        mask = cv2.erode(mask,None,iterations=2)
        mask = cv2.dilate(mask,None,iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	    	cv2.CHAIN_APPROX_SIMPLE)
        # print(cnts)
        # cv2.drawContours(self.cv_image, cnts[0], -1, (0,0,255), 2)
        self.num_switches_detected = 0
        cnts = imutils.grab_contours(cnts)
        if len(cnts)>0:
            self.switches_pixles = np.zeros([2,len(cnts)],dtype=int)
            for i in range(len(cnts)):
                c = max(cnts, key=cv2.contourArea)
                ((xi,yi),radii) = cv2.minEnclosingCircle(cnts[i])
                self.radius = radii
                if [int(yi),int(xi)] in pos_idx:
                    self.num_switches_detected += 1
                    # print(self.num_switches_detected)
                    self.switches_pixles[0][i] = int(xi)
                    self.switches_pixles[1][i] = int(yi)  
                    if radii > 3:
                        # print(i,(xi,yi))
                        cv2.circle(self.cv_image, (int(xi),int(yi)), int(radii),(0,255,0),2)
                        cv2.putText(self.cv_image,'Switch '+str(self.num_switches_detected - 1),(int(xi-radii),int(yi-radii)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),2,cv2.LINE_AA,False)
            

    def get_switch_depth(self):
        xoffsets = [-1,0,1]
        yoffsets = [-1,0,1]
        avg_depth = [0.0,0.0]
        self.switches_pixles = self.switches_pixles.tolist()
        # print(self.switches_pixles,type(self.switches_pixles))
        for i in range(len(self.switches_pixles[0])):
            for xoffset in xoffsets:
                for yoffset in yoffsets:
                    xcenter = max(min(self.switches_pixles[0][i]+xoffset,639),0)
                    ycenter = max(min(self.switches_pixles[1][i]+yoffset,479),0)
                    avg_depth[i] += 0.001*self.depth_image[ycenter][xcenter]
            avg_depth[i] /= len(xoffsets)*len(yoffsets)
        # xdepth = int(self.rgbFOV[0]*float(self.ballloc_pixel[0]-self.center_pixel[0])/self.depthFOV[0]) + int(640/2)
        # ydepth = int(self.rgbFOV[1]*float(self.ballloc_pixel[1]-self.center_pixel[1])/self.depthFOV[1]) + int(480/2)
        # xdepth = max(min(xdepth,639),0)
        # ydepth = max(min(ydepth,479),0)
        xdepth = np.array(self.switches_pixles)[0][:]
        ydepth = np.array(self.switches_pixles)[1][:]
        self.switches_pixle_vec = np.array([xdepth[:self.num_switches_detected],ydepth[:self.num_switches_detected],np.ones(self.num_switches_detected)])
        self.switch_depth = 0.001*self.depth_image[ydepth[:self.num_switches_detected],xdepth[:self.num_switches_detected]]
        # print(self.switch_depth,ydepth,xdepth)
        self.switches_pixle_vec = np.reshape(self.switches_pixle_vec,[3,self.num_switches_detected])
             
    def switch_state(self):
        for i in range(self.num_switches_detected - 1):
            diff = self.switch_depth[i] - self.switch_depth[i+1]
            if diff > 0:
                print(str(i) + ' is pressed')
                cv2.circle(self.cv_image, (self.switches_pixles[0][i], self.switches_pixles[1][i]), int(self.radius),(255,255,0),2)
                cv2.putText(self.cv_image,'PRESSED!',(int(self.switches_pixles[0][i]-self.radius - 10),int(self.switches_pixles[1][i]-self.radius)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,255),2,cv2.LINE_AA,False)
            elif diff < 0:
                print(str(i + 1) + ' is pressed')
                cv2.circle(self.cv_image, (self.switches_pixles[0][i+1], self.switches_pixles[1][i+1]), int(self.radius),(255,255,0),2)
                cv2.putText(self.cv_image,'PRESSED!',(int(self.switches_pixles[0][i+1]-self.radius - 10),int(self.switches_pixles[1][i+1]+self.radius)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,255),2,cv2.LINE_AA,False)


    def get_switches_xyz(self):
        self.get_switch_depth()
        # print("Depth of swicthes: {}".format(self.switch_depth))
        avg_d = np.mean(self.switch_depth)
        print(avg_d)
        if avg_d < 0.05 or avg_d > 20.0:
            return
        self.switch_xyz = np.zeros([3,self.num_switches_detected])
        for i in range(self.num_switches_detected):
            self.switch_xyz[:,i] = (avg_d/self.lamb)*np.matmul(np.matmul(self.Rt,np.linalg.inv(self.K)),self.switches_pixle_vec[:,i])
            print("Switch ",i+1," is at:")
            print(self.switch_xyz[:,i])

    def pub_switch_xyz(self):
        try:
            self.get_switch_pixel()
            # self.camera_transform()
            self.switch_flag = True
            self.get_switches_xyz()
            msg = PoseArray()
            msg.header.frame_id = "/camera_link"
            self.switches_pixles = np.array(self.switches_pixles)
            for i in range(self.num_switches_detected):
                xyz = self.panel_pc(self.switches_pixles[:,i])
                switch_msg = Pose()
                switch_msg.position.x = -xyz[2]
                switch_msg.position.y = xyz[0]
                switch_msg.position.z = -xyz[1]
                # print(switch_msg.position,self.switches_pixles[:,i],'+++++++++')
                # print(i, switch_msg.position)
                # switch_msg.covariance[0] = self.error
                # switch_msg.covariance[8] = self.error
                # switch_msg.covariance[15] = self.error
                msg.poses.append(switch_msg)
                self.switch_pos_pub.publish(msg)
                br = tf.TransformBroadcaster()
                br.sendTransform((xyz[2],-xyz[0],-xyz[1]),(0.0,0.0,0.0,1.0),rospy.Time.now(),"switch_"+str(i),"camera_link")
        except:
            print('No Siwtches detected')
        #     pass

        

    def image_cb(self,data):
        try:
            self.cv_image = bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)
        self.org_img = self.cv_image
        cv_image_hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV)
	    
        mask = cv2.inRange(cv_image_hsv,orange_lower,orange_upper)
        mask = cv2.erode(mask,None,iterations=2)
        mask = cv2.dilate(mask,None,iterations=2)
        self.mask = mask
        

	    # find contours in the mask and initialize the current
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	    	cv2.CHAIN_APPROX_SIMPLE)

        
        cv2.drawContours(self.cv_image, cnts[0], -1, (0,0,255), 2)
        cnts = imutils.grab_contours(cnts)
	    # only proceed if at least one contour was found
        if len(cnts)>0:
            c = max(cnts, key=cv2.contourArea)
            ((x,y),radius) = cv2.minEnclosingCircle(c)
            self.panel_center_pixel = [int(x),int(y)]
            
            if radius > 3:
                cv2.circle(self.cv_image, (int(x),int(y)), int(radius),(0,0,255),2)
                self.radius = radius
            self.biggest_contour = np.array(c) 
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            box[0] = box[0] - np.array([-10,+10])
            box[1] = box[1] - np.array([-10,-10])
            box[2] = box[2] - np.array([10,-10])
            box[3] = box[3] - np.array([10,10])
            cv2.drawContours(self.cv_image,[box], 0 , (0,255,255),2)
            cv2.circle(self.cv_image, tuple(box[0]), radius=8, color=(0, 255, 0), thickness=-1)
            cv2.circle(self.cv_image, tuple(box[1]), radius=8, color=(0, 255, 0), thickness=-1)
            cv2.circle(self.cv_image, tuple(box[2]), radius=8, color=(0, 255, 0), thickness=-1)
            cv2.circle(self.cv_image, tuple(box[3]), radius=8, color=(0, 255, 0), thickness=-1)
            cv2.circle(self.cv_image, tuple(self.panel_center_pixel), radius=8, color=(255, 255, 0), thickness=-1)
            dummy = np.zeros(np.shape(self.mask),dtype=np.uint8)
            
            # self.get_roi(x,y,radius)
            # self.mask = cv2.bitwise_or(self.mask,dummy)

            # *************** Change the area of interest based on these pixels. Select rectangular region minus offset.*************
            self.panel_corners_pixel = np.transpose(np.array(box))
            # self.mask = cv2.dilate(mask,None,iterations=18)
            
            self.masked_image = cv2.bitwise_and(self.cv_image,self.cv_image,mask = self.mask)           
            self.masked_image_neg = cv2.bitwise_or(self.cv_image,self.cv_image,mask = 255 - mask)

            # self.roi = np.zeros(np.shape(self.mask),dtype="uint8")
            # self.roi[100:400][300:1200] = 255
            # self.mask = self.get_roi(box) 
            # self.masked_image_neg = cv2.bitwise_and(self.cv_image,self.cv_image,mask = 255-self.mask)
            # self.roi = cv2.bitwise_or(self.masked_image_neg,self.masked_image_neg,mask = self.mask)

    def get_roi(self,x,y,radius):
        t = np.linspace(0,2*np.pi,num=int(2*np.pi*radius),dtype=np.uint8)
        self.cropped_image = np.zeros(np.shape(self.cv_image))
        Xs = np.array([x])
        Ys = np.array([y])
        for i in range(1,int(radius)):   
            Xs = np.append(Xs, x + i*np.cos(t))
            Ys = np.append(Ys, y + i*np.sin(t))
        Xs = Xs.astype(int)
        Ys = Ys.astype(int)
        midx = min(len(Xs),len(Ys))
        Ys = Ys[:midx]
        Xs = Xs[:midx]
        # print(self.cv_image[300:400,300:400,:])
        self.cropped_image[0:400,0:400,:] = self.org_img[0:400,0:400,:]
        

    

    def show_depth_map(self):
        min_depth = np.min(self.depth_image)
        # min_depth = np.array([5000])
        # print(min_depth,'*********',)
        max_depth = np.max(self.depth_image)
        # max_depth = np.array([50000])
        # print(max_depth,'*********',)
        # print(np.shape(np.where(self.depth_image == min_depth)))
        depth_mask = np.zeros([np.shape(self.depth_image)[0],np.shape(self.depth_image)[1],3])
        depth_mask[:,:,0] = int(0)
        depth_mask[:,:,1] = 255*(self.depth_image[:,:] - min_depth)/(max_depth - min_depth).astype(int)
        depth_mask[:,:,2] = 255*(1-((self.depth_image[:,:] - min_depth)/(max_depth - min_depth))).astype(int)
        # print('kcjhbskdvfhc',type(depth_mask))
        # self.depth_mask = im.fromarray(depth_mask)
        depth_mask = cv2.bitwise_and(depth_mask,depth_mask,mask = self.mask)
        # cv2.imshow('im4',depth_mask)


    def depth_cb(self,data):
        try:
            self.depth_image = bridge.imgmsg_to_cv2(data,"16UC1")
        except CvBridgeError as e:
            print(e)

    def camerainfo_cb(self,data):
        # print("Camera info:")
        self.K = np.reshape(np.array(data.K),[3,3])



if __name__ == "__main__":
    rospy.init_node("measure_3d")
    tracker = Tracker3D()
    tracker.camera_transform()
    viz_img = True
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        # tracker.get_roi(tracker.center_pixel[0],tracker.center_pixel[1],tracker.radius)
        tracker.get_panel_xyz()
        tracker.pub_panel_xyz()
        tracker.pub_viz()
        tracker.pub_switch_xyz()
        # tracker.switch_state()
        # tracker.show_depth_map()
        # print("Ball location: ({},{})".format(tracker.ballloc_xyz[0],tracker.ballloc_xyz[1]))
        if viz_img:
            # tracker.viz_3d()
            cv2.circle(tracker.cv_image, (int(tracker.center_pixel[1]),int(tracker.center_pixel[0])), int(10),(255,0,0),2)
            cv2.imshow("Image window 1",tracker.cv_image)
            # cv2.imshow("Image window 2",tracker.mask)
            # cv2.imshow("Image window 3",tracker.masked_image_neg)
            # cv2.imshow("Image window 3",tracker.cropped_image)
            # cv2.imshow("Image window 3",tracker.depth_image)
            cv2.waitKey(1) & 0xFF
        rate.sleep()
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
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, PoseArray, Pose
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError
import pdb
from PIL import Image as im
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import time
import tf
import tf2_ros
import tf_conversions
import ros_numpy
import random
import time
import door_detect



orange_lower = (5,155,100)
orange_upper = (15,255,255)
red_lower = (1,200,100)
red_upper = (5,255,255)
yellow_lower = (35,70,100)
yellow_upper = (45,120,200) 
bridge = CvBridge()

door_knob_center_x = 0

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
		self.door_handle_xyz = np.zeros([3,])
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
		self.open3d_PC = None
		

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
			br.sendTransform((xyz[2],-xyz[0],-xyz[1]),(0.0,0.0,0.0,1.0),rospy.Time.now(),"door_handle_center","camera_link")

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



	def camerainfo_cb(self,data):
		# print("Camera info:")
		self.K = np.reshape(np.array(data.K),[3,3])


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

	def camera_transform(self):
		br = tf2_ros.StaticTransformBroadcaster()
		t = TransformStamped()

		t.header.stamp = rospy.Time.now()
		t.header.frame_id = tf_prefix + "shoulder_link"
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
		


	def depth_cb(self,data):
		try:
			self.depth_image = bridge.imgmsg_to_cv2(data,"16UC1")
		except CvBridgeError as e:
			print(e)


	def ReadPlyPoint(self, fname):
		""" read point from ply

		Args:
			fname (str): path to ply file

		Returns:
			[ndarray]: N x 3 point clouds
		"""

		pcd = o3d.io.read_point_cloud(fname)

		return self.PCDToNumpy(pcd)


	def NumpyToPCD(self, xyz):
		""" convert numpy ndarray to open3D point cloud 

		Args:
			xyz (ndarray): 

		Returns:
			[open3d.geometry.PointCloud]: 
		"""

		pcd = o3d.geometry.PointCloud()
		pcd.points = o3d.utility.Vector3dVector(xyz)

		return pcd


	def PCDToNumpy(self, pcd):
		"""  convert open3D point cloud to numpy ndarray

		Args:
			pcd (open3d.geometry.PointCloud): 

		Returns:
			[ndarray]: 
		"""

		return np.asarray(pcd.points)


	def RemoveNan(self, points):
		""" remove nan value of point clouds

		Args:
			points (ndarray): N x 3 point clouds

		Returns:
			[ndarray]: N x 3 point clouds
		"""

		return points[~np.isnan(points[:, 0])]


	def RemoveNoiseStatistical(self, pc, nb_neighbors=20, std_ratio=2.0):
		""" remove point clouds noise using statitical noise removal method

		Args:
			pc (ndarray): N x 3 point clouds
			nb_neighbors (int, optional): Defaults to 20.
			std_ratio (float, optional): Defaults to 2.0.

		Returns:
			[ndarray]: N x 3 point clouds
		"""

		pcd = self.NumpyToPCD(pc)
		cl, ind = pcd.remove_statistical_outlier(
			nb_neighbors=nb_neighbors, std_ratio=std_ratio)

		return self.PCDToNumpy(cl)


	def DownSample(self, pts, voxel_size=0.003):
		""" down sample the point clouds

		Args:
			pts (ndarray): N x 3 input point clouds
			voxel_size (float, optional): voxel size. Defaults to 0.003.

		Returns:
			[ndarray]: 
		"""

		p = self.NumpyToPCD(pts).voxel_down_sample(voxel_size=voxel_size)

		return self.PCDToNumpy(p)


	def PlaneRegression(self, points, threshold=0.01, init_n=3, iter=1000):
		""" plane regression using ransac

		Args:
			points (ndarray): N x3 point clouds
			threshold (float, optional): distance threshold. Defaults to 0.003.
			init_n (int, optional): Number of initial points to be considered inliers in each iteration
			iter (int, optional): number of iteration. Defaults to 1000.

		Returns:
			[ndarray, List]: 4 x 1 plane equation weights, List of plane point index
		"""

		pcd = self.NumpyToPCD(points)

		w, index = pcd.segment_plane(
			threshold, init_n, iter)
		print("PlaneRegression")

		return w, index


	def DrawResult(self, points, colors, medians, m_colors):
		pcd = o3d.geometry.PointCloud()
		pcd.points = o3d.utility.Vector3dVector(points)
		pcd.colors = o3d.utility.Vector3dVector(colors)
		pcd_center = o3d.geometry.PointCloud()
		print("type points, colors, medians, mcolors", type(points), type(colors), type(np.array(medians)), type(np.array(m_colors)))
		print("medians" , medians)
		centers = np.array(medians)
		center_colors = np.array(m_colors)
		print("points", points, points.shape)
		print("pcd center", pcd.get_center())
		 
		print("centers shape", centers, centers.shape, len(centers))
		print("center_colors shape", center_colors, center_colors.shape, len(center_colors))
		pcd_center.points = o3d.utility.Vector3dVector(centers)
		#do some filtering here to only get the door handle center and none of the other centers
		#probably by its current y location? :)
		#julia please
		pcd_center.colors = o3d.utility.Vector3dVector(center_colors)
		
		o3d.visualization.draw_geometries([pcd, pcd_center])
	
	def DetectMultiPlanes(self, points, min_ratio=0.05, threshold=0.01, iterations=1000):
		""" Detect multiple planes from given point clouds

		Args:
			points (np.ndarray): 
			min_ratio (float, optional): The minimum left points ratio to end the Detection. Defaults to 0.05.
			threshold (float, optional): RANSAC threshold in (m). Defaults to 0.01.

		Returns:
			[List[tuple(np.ndarray, List)]]: Plane equation and plane point index
		"""

		plane_list = []
		N = len(points)
		target = points.copy()
		count = 0
		largest_plane = 0
		height_of_doorhandle = 12
		#find max x and y, only allow point clouds that have x and ys that are between that. This sounds way more complicated.
		z_of_largest_plane = 12

		while count < (1 - min_ratio) * N:
			#print("N and count is ", N, " ", count)
			w, index = self.PlaneRegression(
				target, threshold=.015, init_n=3, iter=iterations)
		
			count += len(index)
			num_points = len(target[index])
			if(num_points > largest_plane and target[index][0][2] < z_of_largest_plane): #if it is a larger plane, then append it! :')
				print("z of largest plane CHANGED. it is", z_of_largest_plane)
				largest_plane = num_points
				z_of_largest_plane =   target[index][0][2]
				plane_list.append((w, target[index]))
				target = np.delete(target, index, axis=0)
				continue
			print("abs(target[index][0][2] - z_of_largest_plane)", abs(target[index][0][2] - z_of_largest_plane))
			if(abs(target[index][0][2] - z_of_largest_plane) < height_of_doorhandle):
				plane_list.append((w, target[index]))           
				print("w is", w)
			target = np.delete(target, index, axis=0)
		#print("N is ", N)
		return plane_list
	  
	
	def find_center(self, points):
		# print("Camera info:")
		self.K = np.reshape(np.array(data.K),[3,3])



if __name__ == "__main__":
	rospy.init_node("measure_3d")
	tracker = Tracker3D()
	door_handle = door_detect.DoorHandleDetect()
	viz_img = True
	rate = rospy.Rate(5)
	count = 0
	while not rospy.is_shutdown():
		while tracker.currentCloud is None:
			rate.sleep()
		break
	if tracker.currentCloud is not None:
			print("here")
			points = door_handle.RemoveNoiseStatistical(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(tracker.currentCloud), nb_neighbors=50, std_ratio=0.5)
			print("here 2")
			t0 = time.time()
			points = door_handle.DownSample(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(tracker.currentCloud))
			results = door_handle.DetectMultiPlanes(points, min_ratio=0.01, threshold=0.005, iterations=2000)
			print('Time:', time.time() - t0)
			planes = []
			medians = []
			m_colors = []  
			colors = []
			for _, plane in results:
				print("plane")
				r = random.random()
				g = random.random()
				b = random.random()

				color = np.zeros((plane.shape[0], plane.shape[1]))
				color[:, 0] = r
				color[:, 1] = g
				color[:, 2] = b

				


				planes.append(plane)
				
				#if(len(plane) < 15000 and len(plane) > 200):
				median = np.median(plane, axis =0)
				medians.append(median)
				m_color = np.zeros(3)
				m_color[0] = 0.0
				m_color[1] = .99
				m_color[2] = 0.0
				m_colors.append(m_color)
					
				colors.append(color)
			
			planes = np.concatenate(planes, axis=0)
			colors = np.concatenate(colors, axis=0)
			#medians = np.concatenate(medians, axis=0)
			#m_colors = np.concatenate(m_colors, axis=0)

			door_handle.DrawResult(planes, colors, medians, m_colors)
	#while not rospy.is_shutdown():
	   # rate.sleep()

#! /usr/bin/env python3
"""
Python 3 wrapper for identifying objects in images

Requires DLL compilation

Original *nix 2.7: https://github.com/pjreddie/darknet/blob/0f110834f4e18b30d5f101bf8f1724c34b7b83db/python/darknet.py
Windows Python 2.7 version: https://github.com/AlexeyAB/darknet/blob/fc496d52bf22a0bb257300d3c79be9cd80e722cb/build/darknet/x64/darknet.py

@author: Philip Kahn, Aymeric Dujardin
@date: 20180911
"""


# pylint: disable=R, W0401, W0614, W0703
import os
import sys
import time
import logging
import random
from random import randint
import math
import statistics
import getopt
from ctypes import *
import numpy as np
import cv2
#import pyzed.sl as sl
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from math import pi
import roslib
#roslib.load_manifest('my_package')
import rospy
import sys
#import imutils
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

# Get the top-level logger object
log = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

bridge = CvBridge()


def sample(probs):
	s = sum(probs)
	probs = [a/s for a in probs]
	r = random.uniform(0, 1)
	for i in range(len(probs)):
		r = r - probs[i]
		if r <= 0:
			return i
	return len(probs)-1


def c_array(ctype, values):
	arr = (ctype*len(values))()
	arr[:] = values
	return arr


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
		self.pos_pub = rospy.Publisher('doorxyz',PoseWithCovariance,queue_size=10)
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

		#self.listener = tf.TransformListener()
		
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
		print("-----> depth Messages received")

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

	def pub_xyz(self, x, y, z):
		msg = PoseWithCovariance()
		msg.pose.position.x = x
		msg.pose.position.y = y
		msg.pose.position.z = z
		# msg.covariance[15] = 0.0
		self.pos_pub.publish(msg)

	def get_object_depth(self, depth, bounds):
		'''
		Calculates the median x, y, z position of top slice(area_div) of point cloud
		in camera frame.
		Arguments:
			depth: Point cloud data of whole frame.
			bounds: Bounding box for object in pixels.
				bounds[0]: x-center
				bounds[1]: y-center
				bounds[2]: width of bounding box.
				bounds[3]: height of bounding box.

		Return:
			x, y, z: Location of object in meters.
		'''
		area_div = 2

		x_vect = []
		y_vect = []
		z_vect = []

		for j in range(int(bounds[0] - area_div), int(bounds[0] + area_div)):
			for i in range(int(bounds[1] - area_div), int(bounds[1] + area_div)):
				#z = depth[i, j, 2]
				z = self.depth_image[i][j]
				if not np.isnan(z) and not np.isinf(z):
					x_vect.append(i)
					y_vect.append(j)
					z_vect.append(z)
		try:
			x_median = statistics.median(x_vect)
			y_median = statistics.median(y_vect)
			z_median = statistics.median(z_vect)
		except Exception:
			x_median = -1
			y_median = -1
			z_median = -1
			pass

		return x_median, y_median, z_median



	def image_cb(self,data):
		print("here")
		try:
			self.cv_image = bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)

		cv_image_hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV)
	

		

	def depth_cb(self,data):
		try:
			self.depth_image = bridge.imgmsg_to_cv2(data,"16UC1")
		except CvBridgeError as e:
			print(e)

	def camerainfo_cb(self,data):
		self.K = np.reshape(np.array(data.K),[3,3])



class BOX(Structure):
	_fields_ = [("x", c_float),
				("y", c_float),
				("w", c_float),
				("h", c_float)]


class DETECTION(Structure):
	_fields_ = [("bbox", BOX),
				("classes", c_int),
				("prob", POINTER(c_float)),
				("mask", POINTER(c_float)),
				("objectness", c_float),
				("sort_class", c_int),
				("uc", POINTER(c_float)),
				("points", c_int),
				("embeddings", POINTER(c_float)),
				("embedding_size", c_int),
				("sim", c_float),
				("track_id", c_int)]


class IMAGE(Structure):
	_fields_ = [("w", c_int),
				("h", c_int),
				("c", c_int),
				("data", POINTER(c_float))]


class METADATA(Structure):
	_fields_ = [("classes", c_int),
				("names", POINTER(c_char_p))]


#lib = CDLL("/home/pjreddie/documents/darknet/libdarknet.so", RTLD_GLOBAL)
#lib = CDLL("darknet.so", RTLD_GLOBAL)
hasGPU = True
if os.name == "nt":
	cwd = os.path.dirname(__file__)
	os.environ['PATH'] = cwd + ';' + os.environ['PATH']
	winGPUdll = os.path.join(cwd, "yolo_cpp_dll.dll")
	winNoGPUdll = os.path.join(cwd, "yolo_cpp_dll_nogpu.dll")
	envKeys = list()
	for k, v in os.environ.items():
		envKeys.append(k)
	try:
		try:
			tmp = os.environ["FORCE_CPU"].lower()
			if tmp in ["1", "true", "yes", "on"]:
				raise ValueError("ForceCPU")
			else:
				log.info("Flag value '"+tmp+"' not forcing CPU mode")
		except KeyError:
			# We never set the flag
			if 'CUDA_VISIBLE_DEVICES' in envKeys:
				if int(os.environ['CUDA_VISIBLE_DEVICES']) < 0:
					raise ValueError("ForceCPU")
			try:
				global DARKNET_FORCE_CPU
				if DARKNET_FORCE_CPU:
					raise ValueError("ForceCPU")
			except NameError:
				pass
			# log.info(os.environ.keys())
			# log.warning("FORCE_CPU flag undefined, proceeding with GPU")
		if not os.path.exists(winGPUdll):
			raise ValueError("NoDLL")
		lib = CDLL(winGPUdll, RTLD_GLOBAL)
	except (KeyError, ValueError):
		hasGPU = False
		if os.path.exists(winNoGPUdll):
			lib = CDLL(winNoGPUdll, RTLD_GLOBAL)
			log.warning("Notice: CPU-only mode")
		else:
			# Try the other way, in case no_gpu was
			# compile but not renamed
			lib = CDLL(winGPUdll, RTLD_GLOBAL)
			log.warning("Environment variables indicated a CPU run, but we didn't find `" +
						winNoGPUdll+"`. Trying a GPU run anyway.")
else:
	
	lib = CDLL("/home/victor/Husky/husky_ws/src/darknet/libdarknet.so", RTLD_GLOBAL) #TO DO make sure this is named correctly
lib.network_width.argtypes = [c_void_p]
lib.network_width.restype = c_int
lib.network_height.argtypes = [c_void_p]
lib.network_height.restype = c_int

predict = lib.network_predict
predict.argtypes = [c_void_p, POINTER(c_float)]
predict.restype = POINTER(c_float)

if hasGPU:
	set_gpu = lib.cuda_set_device
	set_gpu.argtypes = [c_int]

make_image = lib.make_image
make_image.argtypes = [c_int, c_int, c_int]
make_image.restype = IMAGE

get_network_boxes = lib.get_network_boxes
get_network_boxes.argtypes = [c_void_p, c_int, c_int, c_float, c_float, POINTER(
	c_int), c_int, POINTER(c_int), c_int]
get_network_boxes.restype = POINTER(DETECTION)

make_network_boxes = lib.make_network_boxes
make_network_boxes.argtypes = [c_void_p]
make_network_boxes.restype = POINTER(DETECTION)

free_detections = lib.free_detections
free_detections.argtypes = [POINTER(DETECTION), c_int]

free_ptrs = lib.free_ptrs
free_ptrs.argtypes = [POINTER(c_void_p), c_int]

network_predict = lib.network_predict
network_predict.argtypes = [c_void_p, POINTER(c_float)]

reset_rnn = lib.reset_rnn
reset_rnn.argtypes = [c_void_p]

load_net = lib.load_network
load_net.argtypes = [c_char_p, c_char_p, c_int]
load_net.restype = c_void_p

load_net_custom = lib.load_network_custom
load_net_custom.argtypes = [c_char_p, c_char_p, c_int, c_int]
load_net_custom.restype = c_void_p

do_nms_obj = lib.do_nms_obj
do_nms_obj.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

do_nms_sort = lib.do_nms_sort
do_nms_sort.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

free_image = lib.free_image
free_image.argtypes = [IMAGE]

letterbox_image = lib.letterbox_image
letterbox_image.argtypes = [IMAGE, c_int, c_int]
letterbox_image.restype = IMAGE

load_meta = lib.get_metadata
lib.get_metadata.argtypes = [c_char_p]
lib.get_metadata.restype = METADATA

load_image = lib.load_image_color
load_image.argtypes = [c_char_p, c_int, c_int]
load_image.restype = IMAGE

rgbgr_image = lib.rgbgr_image
rgbgr_image.argtypes = [IMAGE]

predict_image = lib.network_predict_image
predict_image.argtypes = [c_void_p, IMAGE]
predict_image.restype = POINTER(c_float)


def array_to_image(arr):
	import numpy as np
	# need to return old values to avoid python freeing memory
	arr = arr.transpose(2, 0, 1)
	c = arr.shape[0]
	h = arr.shape[1]
	w = arr.shape[2]
	arr = np.ascontiguousarray(arr.flat, dtype=np.float32) / 255.0
	data = arr.ctypes.data_as(POINTER(c_float))
	im = IMAGE(w, h, c, data)
	return im, arr


def classify(net, meta, im):
	out = predict_image(net, im)
	res = []
	for i in range(meta.classes):
		if altNames is None:
			name_tag = meta.names[i]
		else:
			name_tag = altNames[i]
		res.append((name_tag, out[i]))
	res = sorted(res, key=lambda x: -x[1])
	return res


def detect(net, meta, image, thresh=.5, hier_thresh=.5, nms=.45, debug=False):
	#Performs the detection
	
	custom_image = cv2.cvtColor(tracker.cv_image, cv2.COLOR_BGR2RGB)
	custom_image = cv2.resize(custom_image, (lib.network_width(
		net), lib.network_height(net)), interpolation=cv2.INTER_LINEAR)
	im, arr = array_to_image(custom_image)
	num = c_int(0)
	pnum = pointer(num)
	predict_image(net, im)
	dets = get_network_boxes(
		net, tracker.cv_image.shape[1], tracker.cv_image.shape[0], thresh, hier_thresh, None, 0, pnum, 0)
	num = pnum[0]
	if nms:
		do_nms_sort(dets, num, meta.classes, nms)
	res = []
	if debug:
		log.debug("about to range")
	for j in range(num):
		for i in range(meta.classes):
			if dets[j].prob[i] > 0:
				b = dets[j].bbox
				if altNames is None:
					name_tag = meta.names[i]
				else:
					name_tag = altNames[i]
				res.append((name_tag, dets[j].prob[i], (b.x, b.y, b.w, b.h), i))
	res = sorted(res, key=lambda x: -x[1])
	free_detections(dets, num)
	return res


netMain = None
metaMain = None
altNames = None



def generate_color(meta_path):
	'''
	Generate random colors for the number of classes mentioned in data file.
	Arguments:
	meta_path: Path to .data file.

	Return:
	color_array: RGB color codes for each class.
	'''
	random.seed(42)
	with open(meta_path, 'r') as f:
		content = f.readlines()
	class_num = int(content[0].split("=")[1])
	color_array = []
	for x in range(0, class_num):
		color_array.append((randint(0, 255), randint(0, 255), randint(0, 255)))
	return color_array


def init():

	thresh = 0.25
	darknet_path="/home/victor/Husky/husky_ws/src/darknet/"   #TO DO make sure that the dark net path is correct 
	config_path = darknet_path + "cfg/yolo-obj.cfg" #use the data sets from https://github.com/MiguelARD/DoorDetect-Dataset
	weight_path = darknet_path + "yolo-obj.weights"
	meta_path = darknet_path + "cfg/door.data"
	svo_path = None
	zed_id = 0

	""" help_str = 'darknet_zed.py -c cfg/yolo-obj.cfg -w yolo-obj.weights -m cfg/door.data -t 0.25 -s <svo_file> -z <zed_id>' #hard code these variables? they should be coming from the command line argument
	try:
		opts, args = getopt.getopt(
			argv, "hc:w:m:t:s:z:", ["config=", "weight=", "meta=", "threshold=", "svo_file=", "zed_id="])
	except getopt.GetoptError:
		log.exception(help_str)
		sys.exit(2)
	for opt, arg in opts:
		if opt == '-h':
			log.info(help_str)
			sys.exit()
		elif opt in ("-c", "--config"):
			config_path = arg
		elif opt in ("-w", "--weight"):
			weight_path = arg
		elif opt in ("-m", "--meta"):
			meta_path = arg
		elif opt in ("-t", "--threshold"):
			thresh = float(arg)
		elif opt in ("-s", "--svo_file"):
			svo_path = arg
		elif opt in ("-z", "--zed_id"):
			zed_id = int(arg) """

	# Import the global variables. This lets us instance Darknet once,
	# then just call performDetect() again without instancing again
	global metaMain, netMain, altNames  # pylint: disable=W0603
	assert 0 < thresh < 1, "Threshold should be a float between zero and one (non-inclusive)"
	if not os.path.exists(config_path):
		raise ValueError("Invalid config path `" +
						 os.path.abspath(config_path)+"`")
	if not os.path.exists(weight_path):
		raise ValueError("Invalid weight path `" +
						 os.path.abspath(weight_path)+"`")
	if not os.path.exists(meta_path):
		raise ValueError("Invalid data file path `" +
						 os.path.abspath(meta_path)+"`")
	if netMain is None:
		netMain = load_net(config_path.encode(
			"ascii"), weight_path.encode("ascii"), 0, 1)  # batch size = 1
	if metaMain is None:
		metaMain = load_meta(meta_path.encode("ascii"))
	print("made it")
	if altNames is None:
		# In thon 3, the metafile default access craps out on Windows (but not Linux)
		# Read the names file and create a list to feed to detect
		try:
			with open(meta_path) as meta_fh:
				meta_contents = meta_fh.read()
				import re
				match = re.search("names *= *(.*)$", meta_contents,
								  re.IGNORECASE | re.MULTILINE)
				if match:
					result = match.group(1)
				else:
					result = None
				try:
					if os.path.exists(result):
						with open(result) as names_fh:
							names_list = names_fh.read().strip().split("\n")
							altNames = [x.strip() for x in names_list]
				except TypeError:
					pass
		except Exception:
			pass
		


	#cam.close()
	#log.info("\nFINISH")



if __name__ == "__main__":
	rospy.init_node("door_detect")
	tracker = Tracker3D()
	rate = rospy.Rate(50)
	init()
	darknet_path="/home/victor/Husky/husky_ws/src/darknet/"   #TO DO make sure that the dark net path is correct 
	meta_path = darknet_path + "cfg/door.data"
	color_array = generate_color(meta_path)

	while not rospy.is_shutdown():
		start_time = time.time() # start time of the loop
		if True: # check if theres an image first
			image = tracker.cv_image
			depth = tracker.depth_image
			print("depth", depth)

			# Do the detection
			thresh = 0.25
			detections = detect(netMain, metaMain, image, thresh)

			log.info(chr(27) + "[2J"+"**** " + str(len(detections)) + " Results ****")
			for detection in detections:
				label = detection[0]
				confidence = detection[1]
				pstring = label+": "+str(np.rint(100 * confidence))+"%"
				log.info(pstring)
				bounds = detection[2]
				y_extent = int(bounds[3])
				x_extent = int(bounds[2])
				# Coordinates are around the center
				x_coord = int(bounds[0] - bounds[2]/2)
				y_coord = int(bounds[1] - bounds[3]/2)
				#boundingBox = [[x_coord, y_coord], [x_coord, y_coord + y_extent], [x_coord + x_extent, y_coord + y_extent], [x_coord + x_extent, y_coord]]
				thickness = 1
				x, y, z = tracker.get_object_depth(depth, bounds)
				print("x", x)
				print("y", y)
				print("z",z)
				if(label == "handle"):
					tracker.pub_xyz(x , y, z)
					
				distance = math.sqrt(x * x + y * y + z * z)
				distance = "{:.2f}".format(distance)
				cv2.rectangle(image, (x_coord - thickness, y_coord - thickness),
							  (x_coord + x_extent + thickness, y_coord + (18 + thickness*4)),
							  color_array[detection[3]], -1)
				cv2.putText(image, label + " " +  (str(distance) + " m"),
							(x_coord + (thickness * 4), y_coord + (10 + thickness * 4)),
							cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
				cv2.rectangle(image, (x_coord - thickness, y_coord - thickness),
							  (x_coord + x_extent + thickness, y_coord + y_extent + thickness),
							  color_array[detection[3]], int(thickness*2))

			cv2.imshow("ZED", image)
			key = cv2.waitKey(5)
			log.info("FPS: {}".format(1.0 / (time.time() - start_time)))
		else:
			key = cv2.waitKey(5)

		if True:
			# tracker.viz_3d()
			cv2.circle(tracker.cv_image, (int(tracker.center_pixel[0]),int(tracker.center_pixel[1])), int(10),(255,0,0),2)
			cv2.imshow("Image window",tracker.cv_image)
			# cv2.imshow("Image window",tracker.mask)
			cv2.waitKey(1) & 0xFF
		rate.sleep()

	cv2.destroyAllWindows()
	main(sys.argv[1:])
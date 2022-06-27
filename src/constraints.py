#!/usr/bin/python3

from queue import Empty
import queue
from turtle import position
from types import DynamicClassAttribute
import readchar
from sympy import false, true
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose, Point
from std_msgs.msg import Bool
from relaxed_ik_ros1.msg import EEPoseGoals
import transformations as T
from sensor_msgs.msg import PointCloud2
import math
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import ros_numpy

res_constraint = 30 ###########################################################################
acc_constraint = 1 ###########################################################################
dist_constraint = 0.15 ############################################################################
res_thresh = 30 ###############################################################################
acc_thresh = 1 ###############################################################################
dist_thresh = 0.750 ##################################################################################
sample_size = 100 ################################################################################
num_samples = 10 ################################################################################
# dist_min = 1.0 #################################################################################
m = 768 ##########################################################################################
n = 1024 #########################################################################################
muX = n/2 ########################################################################################
muY = m/2 ########################################################################################
sigmaX = muX/3 ###################################################################################
sigmaY = muY/3 ###################################################################################


def sensorCB(data):
    global scan
    global xyz_arrray
    scan = data
    xyz_arrray = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(scan)
    # print(xyz_arrray)

# def locationCB(data):
#     global pose_data
#     pose_data = data

def get_sample():
    if scan is not None:
        try:
            max_x = np.max(xyz_arrray[:,0])
            min_x = np.min(xyz_arrray[:,0])
            max_y = np.max(xyz_arrray[:,1])
            min_y = np.min(xyz_arrray[:,1])

            # print(np.mean(xyz_arrray[:,0]))
            # print(np.std(xyz_arrray[:,0])/2)

            x = np.random.normal(np.mean(xyz_arrray[:,0]),np.std(xyz_arrray[:,0]),1)
            y = np.random.normal(np.mean(xyz_arrray[:,1]),np.std(xyz_arrray[:,1]),1)

            x_start = (x-np.std(xyz_arrray[:,0]))
            x_end = (x+np.std(xyz_arrray[:,0]))
            y_start = (y-np.std(xyz_arrray[:,1]))
            y_end = (y+np.std(xyz_arrray[:,1]))

            # print(x_start)
            # print(x_end)
            # print(y_start)
            # print(y_end)

            # pixels = None
            # for x_p in range(x_start,x_end):
            #     for y_p in range(y_start,y_end):
            #         coordinate = np.array([x_p,y_p])
            #         if pixels is None:
            #             pixels = coordinate
            #         else:
            #             pixels = np.vstack((pixels,coordinate))
            # pixels = pixels.tolist()
            # # print(pixels)
            # gen = pc2.read_points(scan, skip_nans=True, field_names=('x','y','z'), uvs=pixels)
            # # print(gen)
            # sampled = list(gen)
            # # print(sampled)

            # xyz_arrray = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(scan)

            # print(xyz_arrray)

            sampled = xyz_arrray

            mask = (sampled[:,0]>x_start) & (sampled[:,0]<x_end) | (sampled[:,1]>y_start) & (sampled[:,1]<y_end)

            sampled = sampled[mask]
            # print(sampled)

            # print(np.mean(sampled[:,0]))
            # print(np.mean(sampled[:,1]))
            # print(np.mean(sampled[:,2]))
            # print(sampled[:,1])

            return sampled
        except:
            return None
    else:
        return None

def get_resolution(data):
    x = data[:,0]
    y = data[:,1]
    # x = data['x']
    # y = data['y']

    try:

        x_min = np.min(x)
        x_max = np.max(x)

        y_min = np.min(y)
        y_max = np.max(y)

        sa = (x_max-x_min)*1000*(y_max-y_min)*1000
        num_pixels = np.size(x)#*np.size(y)
        # print(np.size(x))
        # print(np.size(y))

        resolution = sa/num_pixels
        return resolution
    except: 
        return 1000

def get_accuracy(data):
    # print(data)
    z = data[:,2]
    # z = data['z']

    std_dev = np.std(z)
    return std_dev

def get_distance(data):
    z = data[:,2]
    # z = data['z']

    z_avg = np.mean(z)
    return z_avg

# def get_location():
#     if pose_data is None:
#         location = false
#     else:
#         position = pose_data.pose.point
#         x = position.x
#         y = position.y
#         z = position.z

#         if last_pose is None:
#             location = true
#         else:
#             dX = last_pose.x-x
#             dY = last_pose.y-y
#             dZ = last_pose.z-z

#             dist = math.sqrt(dX**2+dY**2+dZ**2)

#             if dist>dist_min:
#                 location = true
#             else:
#                 location = false

#     return location

def get_stats():
    accuracy = 0        # standard deviation of z of points (axis coming out of the camera)
    resolution = 0      # distance between points
    distance = 0        # distance from wall

    i = 0
    while i<num_samples:
        sample = get_sample()
        # print(sample)
        if sample is not None: 
            # print(not sample.size==0)
            if not sample.size==0:
                accuracy = accuracy + get_accuracy(sample)
                resolution = resolution + get_resolution(sample)
                distance = distance + get_distance(sample)
            else:
                return false,true
        else:
            return false,true
        i = i+1
    
    avg_acc = accuracy/num_samples
    avg_res = resolution/num_samples
    avg_dist = distance/num_samples

    print("average acc: {}".format(avg_acc))
    print("average res: {}".format(avg_res))
    print("average dist: {}".format(avg_dist))

    if avg_acc<acc_thresh and avg_res<res_thresh and avg_dist<dist_thresh:
        closer = false
    else:
        closer = true
    
    if avg_acc>acc_thresh-acc_constraint and avg_res>res_thresh-res_constraint and avg_dist>dist_thresh-dist_constraint:
        farther = false
    else:
        farther = true

    print("closer: {}".format(closer))
    print("farther: {}".format(farther))

    return closer,farther

def find_constraints():    
    closer,farther = get_stats()
    print("closer: {}".format(closer))
    print("farther: {}".format(farther))

    # if ~(closer or farther):
    #     location = get_location()
    # else:
    #     location = false
    
    return closer,farther

def main():
    global last_pose
    global scan
    # global pose_data

    last_pose = None
    scan = None
    # pose_data = None
 
    rospy.init_node('constraints')
    trigger_pub = rospy.Publisher('/accept_scan',Bool,queue_size=5)

    rospy.Subscriber("/camera/depth/color/points", PointCloud2, sensorCB) ###########################################

    # rospy.Subscriber("chatter". PoseStamped, locationCB) ################################################

    pos_stride = 0.015 ###########################################################################
    rot_stride = 0.055

    position_r = [0,0,0]
    rotation_r = [1,0,0,0]

    closer = false
    farther = false
    # location = false
    
    seq = 1
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        closer,farther = find_constraints()

        if farther:
            rospy.set_param("/direction",1)
            q = Bool()
            q.data = False
            trigger_pub.publish(q)
        elif closer:
            rospy.set_param("/direction",-1)
            q = Bool()
            q.data = False
            trigger_pub.publish(q)
        # elif location:
        #     rospy.set_param("\trigger",true) #####################################################
        #     last_pose = pose_data.pose.point
        else:
            rospy.set_param("/direction",0)
            # rospy.set_param("/constraints",true) #####################################################
            q = Bool()
            q.data = True
            trigger_pub.publish(q)

        rate.sleep()

if __name__ == '__main__':
    main()
#!/usr/bin/python3

import readchar
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose
from std_msgs.msg import Bool
from relaxed_ik_ros1.msg import EEPoseGoals
import transformations as T
from scipy.spatial.transform import Rotation as R
import numpy as np
import tf2_ros
import tf
import tf_conversions

class EE_pose_traj:
    def __init__(self):
        self.ik_goal_r_pub = rospy.Publisher('/ik_goal_r',PoseStamped,queue_size=5)
        self.goal_pos_pub = rospy.Publisher('vive_position', Vector3Stamped,queue_size=5)
        self.goal_quat_pub = rospy.Publisher('vive_quaternion', QuaternionStamped,queue_size=5)
        self.ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=5)
        self.quit_pub = rospy.Publisher('/relaxed_ik/quit',Bool,queue_size=5)

        self.pos_stride = 0.015
        self.rot_stride = 0.055

        self.position_r = [0,0,0]
        self.rotation_r = [1,0,0,0]
        
        self.seq = 1
        
    
    def pose_lookup(self,parent, child):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        rospy.sleep(1)
        
        try:
            trans =  tfBuffer.lookup_transform(parent, child, rospy.Time())
            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(5)

    def local_to_global(self,x=0,y=0,z=0,rx=0,ry=0,rz=0):
        pose_l = self.pose_lookup("starting_pose","ee_link")
        pose_l.transform.translation.z = pose_l.transform.translation.z
        pose_l.transform.rotation.z = pose_l.transform.rotation.z
        print(pose_l)

        r_l = R.from_quat([pose_l.transform.rotation.x,pose_l.transform.rotation.y,pose_l.transform.rotation.z,pose_l.transform.rotation.w])
        print(r_l.as_quat())
        r_lm = r_l.as_matrix()
        r_l4 = np.vstack([r_lm,np.array([0,0,0])])
        t_l = np.array([pose_l.transform.translation.x,pose_l.transform.translation.y,pose_l.transform.translation.z,1]).T
        t_d = np.array([x,y,z,1]).T

        se3 = np.column_stack((r_l4,t_l))

        t_g = np.matmul(se3,t_d)

        r_d = R.from_rotvec([rx,ry,rz])
        # print(r_d.as_rotvec())
        # print(r_l.as_rotvec())
        r_g = r_l.as_rotvec()+r_d.as_rotvec()
        r_g_fixed = r_g
        r_g_fixed[2] = r_g_fixed[2]
        r_g_fixed = R.from_rotvec(r_g_fixed)
        r_g_fixed = -r_g_fixed.as_quat()
        print(t_g)
        print(r_g)


        xg = t_g[0]
        yg = t_g[1]
        zg = -t_g[2]

        rxg = r_g_fixed[0]
        ryg = r_g_fixed[1]
        rzg = r_g_fixed[2]
        rwg = r_g_fixed[3]

        self.move_global(xg,yg,zg,rxg,ryg,rzg,rwg)
        
    def move_global(self,x=0,y=0,z=0,rx=0,ry=0,rz=0,rw=1):
        pose_r = Pose()
        
        pose_r.position.x = x
        pose_r.position.y = y
        pose_r.position.z = z

        # rotation_r = T.quaternion_from_euler(rx,ry,rz)

        pose_r.orientation.w = rw
        pose_r.orientation.x = rx
        pose_r.orientation.y = ry
        pose_r.orientation.z = rz
        print(pose_r)

        ee_pose_goals = EEPoseGoals()

        ee_pose_goals.ee_poses.append(pose_r)

        ee_pose_goals.header.seq = self.seq
        self.seq += 1
        self.ee_pose_goals_pub.publish(ee_pose_goals)

        # self.position_r[0] = x
        # self.position_r[1] = y
        # self.position_r[2] = z

        # self.rotation_r[0] = rx
        # self.rotation_r[1] = ry
        # self.rotation_r[2] = rz
        # self.rotation_r[3] = rw
 
def main():
    rospy.init_node('keyboard_ikgoal_driver')

    rate = rospy.Rate(1000)

    planner = EE_pose_traj()
    print("start")
    while not rospy.is_shutdown():
        key = readchar.readkey()
        if key == 'h':
            planner.move_global()
        elif key == 'w':
            # position_r[0] += pos_stride
            planner.move_global(-0.5)
            rospy.sleep(1)
            planner.move_global(-0.25)
            rospy.sleep(1)
            planner.move_global(0)
            rospy.sleep(1)
            planner.move_global(0.25)
            rospy.sleep(1)
            planner.move_global(0.5)
            # planner.local_to_global(planner.pos_stride,0,0,0,0,0)
        elif key == 'x':
            # position_r[0] -= pos_stride
            # planner.move_global(-0.2524495701754117,-0.2678927883874799,-0.6805315315949477,0.5,-0.5,0.5,-0.5)
            # planner.local_to_global(-planner.pos_stride,0,0,0,0,0)
            planner.move_global(0,-0.5)
            rospy.sleep(1)
            planner.move_global(0,-0.25)
            rospy.sleep(1)
            planner.move_global(0,0)
            rospy.sleep(1)
            planner.move_global(0,0.25)
            rospy.sleep(1)
            planner.move_global(0,0.5)
        elif key == 'a':
            # position_r[1] += pos_stride
            # planner.local_to_global(0,planner.pos_stride,0,0,0,0)
            planner.move_global(0,0,-0.5)
            rospy.sleep(1)
            planner.move_global(0,0,-0.25)
            rospy.sleep(1)
            planner.move_global(0,0,0)
            rospy.sleep(1)
            planner.move_global(0,0,0.25)
            rospy.sleep(1)
            planner.move_global(0,0,0.5)
        elif key == 'd':
            # position_r[1] -= pos_stride
            # planner.local_to_global(0,-planner.pos_stride,0,0,0,0)
            planner.move_global(0.05,0.05,0.05)
        elif key == 'q':
            # position_r[2] += pos_stride
            # planner.local_to_global(0,0,planner.pos_stride,0,0,0)
            planner.move_global(0.05,0.05,-0.05)
        elif key == 'z':
            # position_r[2] -= pos_stride
            # planner.local_to_global(0,0,-planner.pos_stride,0,0,0)
            planner.move_global(0.05,-0.05,-0.05)
        elif key == '1':
            # euler = list(T.euler_from_quaternion(rotation_r))
            # euler[0] += rot_stride
            # rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
            # planner.local_to_global(0,0,0,planner.rot_stride,0,0)
            planner.move_global(-0.05,-0.05,-0.05)
        elif key == '2':
            # euler = list(T.euler_from_quaternion(rotation_r))
            # euler[0] -= rot_stride
            # rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
            # planner.local_to_global(0,0,0,-planner.rot_stride,0,0)
            planner.move_global(-0.05,-0.05,0.05)
        elif key == '3':
            # euler = list(T.euler_from_quaternion(rotation_r))
            # euler[1] += rot_stride
            # rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
            # planner.local_to_global(0,0,0,0,planner.rot_stride,0)
            planner.move_global(-0.05,0.05,0.05)
        elif key == '4':
            # euler = list(T.euler_from_quaternion(rotation_r))
            # euler[1] -= rot_stride
            # rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
            # planner.local_to_global(0,0,0,0,-planner.rot_stride,0)
            planner.move_global(0.05,-0.05,0.05)
        elif key == '5':
            # euler = list(T.euler_from_quaternion(rotation_r))
            # euler[2] += rot_stride
            # rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
            # planner.local_to_global(0,0,0,0,0,planner.rot_stride)
            planner.move_global(-0.05,0.05,-0.05)
        elif key == '6':
            # euler = list(T.euler_from_quaternion(rotation_r))
            # euler[2] -= rot_stride
            # rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
            planner.local_to_global(0,0,0,0,0,-planner.rot_stride)

        elif key == 'q':
            q = Bool()
            q.data = True
            planner.quit_pub.publish(q)
        elif key == 'c':
            rospy.signal_shutdown()   

        q = Bool()
        q.data = False
        planner.quit_pub.publish(q)

        rate.sleep()

if __name__ == '__main__':
    main()
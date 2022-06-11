#!/usr/bin/python3

import readchar
import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose
from std_msgs.msg import Bool
from relaxed_ik_ros1.msg import EEPoseGoals
import transformations as T

rospy.init_node('keyboard_ikgoal_driver')

ik_goal_r_pub = rospy.Publisher('/ik_goal_r',PoseStamped,queue_size=5)
ik_goal_l_pub = rospy.Publisher('/ik_goal_l',PoseStamped,queue_size=5)
goal_pos_pub = rospy.Publisher('vive_position', Vector3Stamped,queue_size=5)
goal_quat_pub = rospy.Publisher('vive_quaternion', QuaternionStamped,queue_size=5)
ee_pose_goals_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=5)
quit_pub = rospy.Publisher('/relaxed_ik/quit',Bool,queue_size=5)

pos_stride = 0.015
rot_stride = 0.055

position_r = [0,0,0]
rotation_r = [1,0,0,0]

position_l = [0,0,0]
rotation_l = [1,0,0,0]

count = 1
seq = 1
key = 0
rospy.set_param("/direction",0)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    pose = PoseStamped()
    pose.pose.position.x = position_r[0]
    pose.pose.position.y = position_r[1]
    pose.pose.position.z = position_r[2]

    pose.pose.orientation.w = rotation_r[0]
    pose.pose.orientation.x = rotation_r[1]
    pose.pose.orientation.y = rotation_r[2]
    pose.pose.orientation.z = rotation_r[3]
    ik_goal_r_pub.publish(pose)

    pose = PoseStamped()
    pose.pose.position.x = position_l[0]
    pose.pose.position.y = position_l[1]
    pose.pose.position.z = position_l[2]

    pose.pose.orientation.w = rotation_l[0]
    pose.pose.orientation.x = rotation_l[1]
    pose.pose.orientation.y = rotation_l[2]
    pose.pose.orientation.z = rotation_l[3]
    ik_goal_l_pub.publish(pose)

    # print("Pos R: {}, Rot R: {}".format(position_r, rotation_r))

    #  key = readchar.readkey()
    if rospy.has_param("/direction"):
        key = rospy.get_param("/direction")
    
    print(key)

    if key == 'w':
        position_r[0] += pos_stride
    elif key == 'x':
        position_r[0] -= pos_stride
    elif key == 'a':
        position_r[1] += pos_stride
    elif key == 'd':
        position_r[1] -= pos_stride
    elif key == 'q':
        position_r[2] += pos_stride
    elif key == 'z':
        position_r[2] -= pos_stride
    elif key == -1:
        position_r[2] += pos_stride
    elif key == 1:
        position_r[2] -= pos_stride

    if position_r[2] < -0.24:
        position_r[2] = -0.24
    # elif key == '1':
    #     euler = list(T.euler_from_quaternion(rotation_r))
    #     euler[0] += rot_stride
    #     rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
    # elif key == '2':
    #     euler = list(T.euler_from_quaternion(rotation_r))
    #     euler[0] -= rot_stride
    #     rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
    # elif key == '3':
    #     euler = list(T.euler_from_quaternion(rotation_r))
    #     euler[1] += rot_stride
    #     rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
    # elif key == '4':
    #     euler = list(T.euler_from_quaternion(rotation_r))
    #     euler[1] -= rot_stride
    #     rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
    # elif key == '5':
    #     euler = list(T.euler_from_quaternion(rotation_r))
    #     euler[2] += rot_stride
    #     rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
    # elif key == '6':
    #     euler = list(T.euler_from_quaternion(rotation_r))
    #     euler[2] -= rot_stride
    #     rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])

    elif key == 'i':
        position_l[0] += pos_stride # left
    elif key == 'm':
        position_l[0] -= pos_stride
    elif key == 'j':
        position_l[1] += pos_stride
    elif key == 'l':
        position_l[1] -= pos_stride
    elif key == 'u':
        position_l[2] += pos_stride
    elif key == 'n':
        position_l[2] -= pos_stride
    elif key == '=':
        euler = list(T.euler_from_quaternion(rotation_l))
        euler[0] += rot_stride
        rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
    elif key == '-':
        euler = list(T.euler_from_quaternion(rotation_l))
        euler[0] -= rot_stride
        rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
    elif key == '0':
        euler = list(T.euler_from_quaternion(rotation_l))
        euler[1] += rot_stride
        rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
    elif key == '9':
        euler = list(T.euler_from_quaternion(rotation_l))
        euler[1] -= rot_stride
        rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
    elif key == '8':
        euler = list(T.euler_from_quaternion(rotation_l))
        euler[2] += rot_stride
        rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
    elif key == '7':
        euler = list(T.euler_from_quaternion(rotation_l))
        euler[2] -= rot_stride
        rotation_l = T.quaternion_from_euler(euler[0],euler[1],euler[2])
    elif key == 'q':
        q = Bool()
        q.data = True
        quit_pub.publish(q)
    elif key == 'c':
        rospy.signal_shutdown()
    
    #if key == 'w':
    #    position_r[0] += pos_stride
    #elif key == 'x':
    #    position_r[0] -= pos_stride
    #elif key == 'a':
    #    position_r[1] += pos_stride
    #elif key == 'd':
    #    position_r[1] -= pos_stride
    #elif key == 'q':
    #    position_r[2] += pos_stride
    #elif key == 'z':
    #    position_r[2] -= pos_stride
    
    if seq == 1:
        
        #for a in range(120):
        #    position_r[1] += pos_stride
        #for x in range(130):
        #    position_r[0] -= pos_stride
        #for d in range(70):
        #    position_r[1] -= pos_stride
        #for six in range(60):
        #    euler = list(T.euler_from_quaternion(rotation_r))
        #    euler[2] -= rot_stride
        #    rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
        if count == 1:
            for x in range(20):
                position_r[1] -= pos_stride #d
                position_r[0] -= pos_stride #x
            count = count+1
        elif count == 2:
            for x in range(15): #6
                euler = list(T.euler_from_quaternion(rotation_r))
                euler[2] -= rot_stride
                rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
            count = count+1
        elif count == 3:
            for x in range(20):
                position_r[1] -= pos_stride #d
                position_r[0] -= pos_stride #x
            count = count+1
        elif count == 4:
            for x in range(15):
                position_r[1] += pos_stride #a
            count = count+1
        elif count == 5:
            for x in range(15):
                position_r[0] -= pos_stride #x
            count = count+1
        elif count == 6:
            for x in range(20): #6
                euler = list(T.euler_from_quaternion(rotation_r))
                euler[2] -= rot_stride
                rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
            count = count+1
        elif count == 7:
            for x in range(25):
                position_r[1] += pos_stride #a
                position_r[0] -= pos_stride #x
            count = count+1
        elif count == 8:
            for x in range(15): #6
                euler = list(T.euler_from_quaternion(rotation_r))
                euler[2] -= rot_stride
                rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
            count = count+1
        elif count == 9:
            for x in range(10):
                position_r[1] += pos_stride #a
                position_r[0] += pos_stride #w
            count = count+1    
        elif count == 10:
            for x in range(5): #6
                euler = list(T.euler_from_quaternion(rotation_r))
                euler[2] -= rot_stride
                rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
            count = count+1
        elif count == 11:
            for x in range(15):
                position_r[2] -= pos_stride #z
            count = count+1
        elif count == 12:
            for x in range(6):
                position_r[1] += pos_stride #a
                position_r[0] += pos_stride #w
            count = count+1 
        elif count == 13:
            for x in range(4): #6
                euler = list(T.euler_from_quaternion(rotation_r))
                euler[2] -= rot_stride
                rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
            count = count+1
            
        elif count == 14:
            for x in range(29):
                euler = list(T.euler_from_quaternion(rotation_r))
                euler[0] += rot_stride
                rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])

                position_saved = position_r
                rotation_saved = rotation_r
                count = 1
            seq = 2

        rospy.sleep(5)

        # elif count == 14:
        #     print("Moving Forward")
        #     for x in range(23):
        #         position_r[2] += pos_stride #q
        #     count = count+1

        # elif count == 15:
        #     print("Moving Backwards")
        #     for x in range(23):
        #         position_r[2] -= pos_stride #z
        #     count = count+1

        
        print(count)

    #if seq == 1:
        #position_r = [-0.9600000000000007, 0.18, -0.30000000000000016]
        #rotation_r = [ 3.07914591e-02 -1.38712074e-17 -4.27317335e-19 -9.99525831e-01]

        #seq=seq+1
        #print("here")
    #elif seq == 2:
    #    position_r = [0,0,0]
    #    rotation_r = [1,0,0,0]
    #    seq=seq+1
    #    print("now here")
    up = False
    if rospy.has_param("/move_up"):
        up = rospy.get_param("/move_up")
    if up:
        if count==1:
            position_r = position_saved
            rotation_r = rotation_saved
            count = count+1
            print("home")
        elif count==2:
            for x in range(25):
                position_r[1] -= pos_stride
                position_r[0] -= pos_stride
            for x in range(18):
                euler = list(T.euler_from_quaternion(rotation_r))
                euler[2] += rot_stride
                rotation_r = T.quaternion_from_euler(euler[0],euler[1],euler[2])
            print("up")
            count = count+1

    print(seq)

    pose = PoseStamped()
    pose.pose.position.x = position_r[0]
    pose.pose.position.y = position_r[1]
    pose.pose.position.z = position_r[2]

    pose.pose.orientation.w = rotation_r[0]
    pose.pose.orientation.x = rotation_r[1]
    pose.pose.orientation.y = rotation_r[2]
    pose.pose.orientation.z = rotation_r[3]
    ik_goal_r_pub.publish(pose)


    pose = PoseStamped()
    pose.pose.position.x = position_l[0]
    pose.pose.position.y = position_l[1]
    pose.pose.position.z = position_l[2]

    pose.pose.orientation.w = rotation_l[0]
    pose.pose.orientation.x = rotation_l[1]
    pose.pose.orientation.y = rotation_l[2]
    pose.pose.orientation.z = rotation_l[3]
    ik_goal_l_pub.publish(pose)

    pos_goal = Vector3Stamped()
    pos_goal.vector.x = position_r[0]
    pos_goal.vector.y = position_r[1]
    pos_goal.vector.z = position_r[2]
    goal_pos_pub.publish(pos_goal)

    quat_goal = QuaternionStamped()
    quat_goal.quaternion.w = rotation_r[0]
    quat_goal.quaternion.x = rotation_r[1]
    quat_goal.quaternion.y = rotation_r[2]
    quat_goal.quaternion.z = rotation_r[3]
    goal_quat_pub.publish(quat_goal)

    ee_pose_goals = EEPoseGoals()
    pose_r = Pose()
    pose_r.position.x = position_r[0]
    pose_r.position.y = position_r[1]
    pose_r.position.z = position_r[2]

    pose_r.orientation.w = rotation_r[0]
    pose_r.orientation.x = rotation_r[1]
    pose_r.orientation.y = rotation_r[2]
    pose_r.orientation.z = rotation_r[3]

    pose_l = Pose()
    pose_l.position.x = position_l[0]
    pose_l.position.y = position_l[1]
    pose_l.position.z = position_l[2]

    pose_l.orientation.w = rotation_l[0]
    pose_l.orientation.x = rotation_l[1]
    pose_l.orientation.y = rotation_l[2]
    pose_l.orientation.z = rotation_l[3]
    ee_pose_goals.ee_poses.append(pose_r)
    ee_pose_goals.ee_poses.append(pose_l)

    ee_pose_goals.header.seq = seq
    #seq += 1
    ee_pose_goals_pub.publish(ee_pose_goals)

    q = Bool()
    q.data = False
    quit_pub.publish(q)

    rate.sleep()



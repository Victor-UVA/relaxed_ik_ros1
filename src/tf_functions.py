import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler


def pose_lookup(parent, child):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rospy.sleep(1)

    try:
        trans = tfBuffer.lookup_transform(parent, child, rospy.Time())
        return trans
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.sleep(5)


def transform(pose, parent, child, x, y, z):
    static_transformStamped = TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = parent
    static_transformStamped.child_frame_id = child

    static_transformStamped.transform.translation.x = float(
        pose.transform.translation.x)
    static_transformStamped.transform.translation.y = float(
        pose.transform.translation.y)
    static_transformStamped.transform.translation.z = float(
        pose.transform.translation.z)

    quat = quaternion_from_euler(float(x), float(y), float(z))
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    return static_transformStamped

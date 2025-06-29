import rospy
import tf2_ros
import geometry_msgs.msg
import time
import numpy as np


rospy.init_node('ee_pose_listener')

tf_buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tf_buffer)

base_frame = 'panda_link0'
ee_frame = 'panda_hand_tcp'

rate = rospy.Rate(10.0)  # Hz

x, y, z = [], [], []
while not rospy.is_shutdown():
    try:
        # Lookup the transform at the latest available time
        trans = tf_buffer.lookup_transform(base_frame, ee_frame, rospy.Time(0), rospy.Duration(1.0))
        pos = trans.transform.translation
        x.append(pos.x)
        y.append(pos.y)
        z.append(pos.z)


    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        rospy.logwarn("Transform not available yet.")
        continue

    rate.sleep()
np.savez_compressed('robot_traj.npz', x=x, y=y, z=z)


import rospy
import tf2_ros
import geometry_msgs.msg
import time


def main():
    time.sleep(3)
    rospy.init_node('ee_pose_listener')

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    base_frame = 'panda_link0'
    ee_frame = 'panda_hand'

    rate = rospy.Rate(10.0)  # Hz
    
    xs, ys, zs = [], [], []
    while not rospy.is_shutdown():
        try:
            # Lookup the transform at the latest available time
            trans = tf_buffer.lookup_transform(base_frame, ee_frame, rospy.Time(0), rospy.Duration(1.0))
            pos = trans.transform.translation

            xs.append(pos.x)
            ys.append(pos.y)
            zs.append(pos.z)
    

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logwarn("Transform not available yet.")
            continue

        rate.sleep()
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(xs, ys, zs, label='EE trajectory')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()

if __name__ == '__main__':
    main()

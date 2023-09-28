#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

def get_end_effector_pose():
    rospy.init_node('get_end_effector_pose')
    
    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))  # tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(1.0)  # 1 Hz
    while not rospy.is_shutdown():
        try:
            # Get the transform from the base to the end-effector
            trans = tf_buffer.lookup_transform('base_link', 'end_effector_link', rospy.Time(0), rospy.Duration(1.0))
            rospy.loginfo("End Effector Pose: %s", trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)
        rate.sleep()

if __name__ == '__main__':
    get_end_effector_pose()


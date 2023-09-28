#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def pose_publisher():
    pub = rospy.Publisher('target_pose', PoseStamped, queue_size=10)
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(1) # 1 Hz
    
    while not rospy.is_shutdown():
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.header.stamp = rospy.Time.now()
        
        # A small example pose change
        pose.pose.position.x = 0.5
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        
        rospy.loginfo(pose)
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        pose_publisher()
    except rospy.ROSInterruptException:
        pass


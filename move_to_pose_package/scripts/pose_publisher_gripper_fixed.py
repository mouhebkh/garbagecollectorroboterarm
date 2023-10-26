#!/usr/bin/env python

import rospy
from move_to_pose_package.msg import JointAngles

def pose_publisher():
    rospy.init_node('pose_publisher', anonymous=True)
    pub = rospy.Publisher('joint_angles_topic', JointAngles, queue_size=10)

    while not rospy.is_shutdown():
        try:
            angles_input = input("Enter joint angles separated by spaces: ")
            angles_list = [float(angle) for angle in angles_input.split()]
            
            gripper_input = int(input("Enter gripper command (1 for open, 0 for close): "))

            joint_angles_msg = JointAngles()
            joint_angles_msg.angles = angles_list
            joint_angles_msg.gripper_command = gripper_input

            pub.publish(joint_angles_msg)
        except Exception as e:
            rospy.logwarn("Error in input: %s", e)

if __name__ == '__main__':
    try:
        pose_publisher()
    except rospy.ROSInterruptException:
        pass


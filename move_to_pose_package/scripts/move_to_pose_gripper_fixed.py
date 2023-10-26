#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from move_to_pose_package.msg import JointAngles

class MoveToPose:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_to_pose', anonymous=True)

        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        rospy.loginfo("MoveGroupCommander objects created successfully.")
        
        # Subscribe to the joint_angles_topic
        rospy.Subscriber("joint_angles_topic", JointAngles, self.joint_angles_callback)
        
        rospy.spin()

    def move_to_target(self, joint_values):
        rospy.loginfo("Planning movement to joint values: %s", joint_values)
        self.arm_group.set_joint_value_target(joint_values)
        
        try:
            success = self.arm_group.go(wait=True)
        except Exception as e:
            rospy.logerr(e)
            return

        self.arm_group.stop()

        if not success:
            rospy.logerr("Execution failed.")
        else:
            rospy.loginfo("Execution successful.")

    def control_gripper(self, command):
        if command == 1:
            # Logic to open the gripper; adjust based on your SRDF file
            self.gripper_group.set_joint_value_target([0.009, 0])  
        elif command == 0:
            # Logic to close the gripper; adjust based on your SRDF file
            self.gripper_group.set_joint_value_target([-0.009, 0]) 
	    # Set velocity scaling factor to slow down the gripper
        self.gripper_group.set_max_velocity_scaling_factor(0.5)
        
        # Execute the gripper command
        try:
            success = self.gripper_group.go(wait=True)
        except Exception as e:
            rospy.logerr(e)
            return

        self.gripper_group.stop()

        if not success:
            rospy.logerr("Gripper execution failed.")
        else:
            rospy.loginfo("Gripper execution successful.")

    def joint_angles_callback(self, msg):
        self.move_to_target(msg.angles)
        self.control_gripper(msg.gripper_command)

if __name__ == '__main__':
    try:
        MoveToPose()
    except rospy.ROSInterruptException:
        pass


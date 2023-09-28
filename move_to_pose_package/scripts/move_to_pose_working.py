#!/usr/bin/env python

import sys
import rospy
import moveit_commander

class MoveToPose:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_to_pose', anonymous=True)

        self.move_group = moveit_commander.MoveGroupCommander("arm")

        rospy.loginfo("MoveGroupCommander object created successfully.")

        self.move_to_target()

    def move_to_target(self):
        rospy.loginfo("Planning movement...")

        self.move_group.set_joint_value_target([-1.134, -0.436, -0.157, 1.957])
        
        try:
            success = self.move_group.go(wait=True)  # Changed this line
        except Exception as e:
            rospy.logerr(e)
            return

        self.move_group.stop()

        if not success:
            rospy.logerr("Execution failed.")
        else:
            rospy.loginfo("Execution successful.")

        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        MoveToPose()
    except rospy.ROSInterruptException:
        pass


#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class MoveToPose:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_to_pose_node', anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        rospy.loginfo("MoveGroupCommander object created successfully.")
        
        self.move_to_target()
    
    def move_to_target(self):
        rospy.loginfo("Planning movement...")

        joint_goal = self.move_group.get_current_joint_values()
        
        joint_goal[0] = -65 * pi / 180
        joint_goal[1] = -25 * pi / 180
        joint_goal[2] = -9 * pi / 180
        joint_goal[3] = 112 * pi / 180

        self.move_group.go(joint_goal, wait=True)
        
        plan = self.move_group.plan()
        
        success = False
        if len(plan.joint_trajectory.points) > 0:
            rospy.loginfo("Plan created successfully.")
            success = self.move_group.execute(plan, wait=True)
        
        if success:
            rospy.loginfo("Movement executed successfully.")
        else:
            rospy.logwarn("Failed to execute movement.")


if __name__ == '__main__':
    try:
        MoveToPose()
    except rospy.ROSInterruptException:
        pass


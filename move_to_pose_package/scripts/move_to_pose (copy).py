#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

class MoveToPose:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_to_pose', anonymous=True)
        
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        
        group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        
        rospy.loginfo("MoveGroupCommander object created successfully.")
        
        self.pose_subscriber = rospy.Subscriber('/target_pose', PoseStamped, self.move_to_target)
        
        rospy.loginfo(f"Subscribing to topic: /target_pose")
        
        rospy.spin()
        rospy.loginfo("Spinning...")

    def move_to_target(self, data):
        rospy.loginfo("Received target pose data.")
        rospy.loginfo(data)
        
        self.move_group.set_pose_target(data.pose)
        
        rospy.loginfo("Target pose set successfully.")
        
        rospy.loginfo("Planning movement...")
        
        plan = self.move_group.plan()
        
        if not plan.joint_trajectory.points:
            rospy.logerr("Planning failed.")
            return
        
        rospy.loginfo("Plan created successfully.")
        
        rospy.loginfo("Executing plan...")
        self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
        rospy.loginfo("Execution completed.")

if __name__ == '__main__':
    try:
        MoveToPose()
    except rospy.ROSInterruptException:
        pass


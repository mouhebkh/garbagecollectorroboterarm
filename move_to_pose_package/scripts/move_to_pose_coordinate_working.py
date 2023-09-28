#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

class MoveToPose:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Initialize the ROS node
        rospy.init_node('move_to_pose_node', anonymous=True)
        
        # Initialize the MoveIt! commander for the arm
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        
        # Set the reference frame
        reference_frame = "base_link"
        self.move_group.set_pose_reference_frame(reference_frame)
        
        # Allow replanning to increase the odds of a solution being found
        self.move_group.allow_replanning(True)

    def go_to_pose_goal(self):
        # Define the target pose (modify these values with reachable ones)
        target_pose = Pose()
        target_pose.position.x = 0.1
        target_pose.position.y = 0.0
        target_pose.position.z = 0.2
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 1.0
        
        # Set the target pose
        self.move_group.set_pose_target(target_pose)
        
        # Plan and execute
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

def main():
    try:
        tutorial = MoveToPose()
        
        print("============ Press `Enter` to execute a movement using a pose goal ...")
        input()
        tutorial.go_to_pose_goal()
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()


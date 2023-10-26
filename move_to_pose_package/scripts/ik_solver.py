
#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

class IKSolver:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Initialize the ROS node
        rospy.init_node('ik_solver_node', anonymous=True)
        
        # Initialize the MoveIt! commander for the arm
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        
        # Set the reference frame
        reference_frame = "world"
        self.move_group.set_pose_reference_frame(reference_frame)
        
        # Allow replanning to increase the odds of a solution being found
        self.move_group.allow_replanning(True)

    def compute_ik(self, pose_target):
        # Set the target pose
        self.move_group.set_pose_target(pose_target)
        
        # Plan
        plan_success = self.move_group.plan()
        
        # If plan was successful, extract the joint values from the move_group's current joint values
        if plan_success:
            joint_values = self.move_group.get_current_joint_values()
            return joint_values
        else:
            return None

def main():
    solver = IKSolver()
    
    # Define a desired pose (you can modify these values)
    pose_target = PoseStamped()
    pose_target.header.frame_id = "world"
    pose_target.pose.position.x = 0.04
    pose_target.pose.position.y = 0.04
    pose_target.pose.position.z = 0.20
    pose_target.pose.orientation.x = 0.0
    pose_target.pose.orientation.y = 0.0
    pose_target.pose.orientation.z = 0.0
    pose_target.pose.orientation.w = 1.0
    
    joint_values = solver.compute_ik(pose_target)
    
    if joint_values:
        print("Joint values for the desired pose:", joint_values)
    else:
        print("Failed to compute joint values for the desired pose.")

if __name__ == '__main__':
    main()

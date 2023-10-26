#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker

class MoveToPose:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Initialize the ROS node
        rospy.init_node('move_to_pose_node', anonymous=True)
        
        # Initialize the MoveIt! commander for the arm
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        
        # Set the reference frame
        reference_frame = "world"
        self.move_group.set_pose_reference_frame(reference_frame)
        
        # Allow replanning to increase the odds of a solution being found
        self.move_group.allow_replanning(True)
        
        # Publisher for the target pose visualization in Rviz
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    def visualize_target_pose(self, pose_target):
        # Publish the target pose as a marker in Rviz
        marker = Marker()
        marker.header.frame_id = pose_target.header.frame_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose_target.pose
        marker.scale.x = 0.1
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0
        self.marker_pub.publish(marker)

    def go_to_pose_goal(self):
        # Define the target pose (modify these values with reachable ones)
        pose_target = PoseStamped()
        pose_target.header.frame_id = "world"
        pose_target.pose.position.x = 0.1
        pose_target.pose.position.y = 0.0
        pose_target.pose.position.z = 0.2
        pose_target.pose.orientation.x = 0.0
        pose_target.pose.orientation.y = 0.0
        pose_target.pose.orientation.z = 0.0
        pose_target.pose.orientation.w = 1.0
        
        # Visualize the target pose in Rviz
        self.visualize_target_pose(pose_target)
        
        # Set the target pose
        self.move_group.set_pose_target(pose_target)
        
        # Plan and execute
        plan = self.move_group.go(wait=True)
        
        # Log the plan status and details
        if plan:
            rospy.loginfo("Plan successful!")
        else:
            rospy.logwarn("Plan failed!")
            rospy.logwarn("Unable to solve the planning problem")
        
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


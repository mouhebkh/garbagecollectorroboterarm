#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from actionlib import SimpleActionClient
from moveit_msgs.msg import MoveGroupAction

class MoveToPose:
    def __init__(self):
        rospy.loginfo("Initializing MoveToPose Node...")
        rospy.init_node('move_to_pose_node', anonymous=True)
        rospy.loginfo("Node initialized.")
        
        # Wait for the move_group action server to be available
        client = SimpleActionClient('move_group', MoveGroupAction)
        rospy.loginfo("Waiting for move_group action server...")
        client.wait_for_server()
        rospy.loginfo("Connected to move_group action server")

        group_name = "arm"
        rospy.loginfo(f"Creating MoveGroupCommander object for group: {group_name}")
        self.group = MoveGroupCommander(group_name)
        rospy.loginfo("MoveGroupCommander object created successfully.")
        
        topic_name = "/target_pose"
        rospy.loginfo(f"Subscribing to topic: {topic_name}")
        rospy.Subscriber(topic_name, PoseStamped, self.move_to_target)
        rospy.loginfo("Subscription successful.")

    def move_to_target(self, data):
        rospy.loginfo("Received target pose data.")
        rospy.loginfo(data)
        
        self.group.set_pose_target(data.pose)
        rospy.loginfo("Target pose set successfully.")
        
        rospy.loginfo("Planning movement...")
        plan = self.group.plan()
        rospy.loginfo(f"Plan created: {plan}")
        
        if not plan.joint_trajectory.points:
            rospy.logwarn("Plan is empty. Unable to move to the target pose.")
            return

        rospy.loginfo("Executing...")
        success = self.group.execute(plan, wait=True)
        rospy.loginfo(f"Movement executed {'successfully' if success else 'unsuccessfully'}.")


    def spin(self):
        rospy.loginfo("Spinning...")
        rospy.spin()
        rospy.loginfo("Node stopped spinning.")

if __name__ == "__main__":
    try:
        rospy.loginfo("Starting MoveToPose Node...")
        moveit = MoveToPose()
        moveit.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROSInterruptException encountered.")
    except Exception as e:
        rospy.logerr(f"Unhandled exception: {str(e)}")


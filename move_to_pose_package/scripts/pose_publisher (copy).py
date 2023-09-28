#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

def main():
    # Initialize the node
    rospy.init_node('pose_publisher_node', anonymous=True)
    
    # Create a publisher for the target pose topic
    pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=10)
    
    rate = rospy.Rate(1)  # 1 Hz
    
    while not rospy.is_shutdown():
        pose = PoseStamped()
        
        # Input position (x, y, z)
        try:
            pose.pose.position.x = float(input("Enter x coordinate: "))
            pose.pose.position.y = float(input("Enter y coordinate: "))
            pose.pose.position.z = float(input("Enter z coordinate: "))
            
            # Input orientation (roll, pitch, yaw)
            roll  = float(input("Enter roll (in radians): "))
            pitch = float(input("Enter pitch (in radians): "))
            yaw   = float(input("Enter yaw (in radians): "))
            
            # Convert roll, pitch, yaw to quaternion
            q = quaternion_from_euler(roll, pitch, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            
            # Publish the pose
            pub.publish(pose)
            
        except ValueError:
            rospy.logerr("Invalid input. Please enter a valid number.")
        
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


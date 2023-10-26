#!/usr/bin/env python
import sys
import rospy
import moveit_commander

def gripper_control():
    rospy.init_node('gripper_control', anonymous=True)
    gripper_group = moveit_commander.MoveGroupCommander("gripper")

    while not rospy.is_shutdown():
        try:
            command = int(input("Enter 1 to open gripper, 0 to close gripper, or -1 to exit: "))
            
            if command == -1:
                break
            
            if command == 1:
                gripper_group.set_joint_value_target([0.01])
            elif command == 0:
                gripper_group.set_joint_value_target([-0.01])
            else:
                rospy.logwarn("Invalid command. Please enter 1 to open, 0 to close, or -1 to exit.")
                continue
            
            success = gripper_group.go(wait=True)
            gripper_group.stop()

            if not success:
                rospy.logerr("Gripper execution failed.")
            else:
                rospy.loginfo("Gripper execution successful.")

        except Exception as e:
            rospy.logwarn("Error: %s", e)

if __name__ == '__main__':
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        gripper_control()
    except rospy.ROSInterruptException:
        pass


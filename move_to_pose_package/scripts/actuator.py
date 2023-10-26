#!/usr/bin/env python

import rospy
import subprocess
import time
from open_manipulator_msgs.srv import SetActuatorState, SetActuatorStateRequest

def set_actuator_state(enable):
    rospy.init_node('actuator_controller', anonymous=True)
    rospy.wait_for_service('/set_actuator_state')
    try:
        set_state = rospy.ServiceProxy('/set_actuator_state', SetActuatorState)
        request = SetActuatorStateRequest()
        request.set_actuator_state = enable
        response = set_state(request)
        return response.is_planned
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def stop_all_ros_nodes():
    subprocess.run(["rosnode", "kill", "-a"])
    time.sleep(1)  # Wait for the nodes to terminate

def is_launch_file_running():
    # This function checks if a specific launch file's nodes are running
    key_node = "/open_manipulator_controller"  # Here, we specify the actual node name
    try:
        subprocess.check_output(["rosnode", "info", key_node], stderr=subprocess.STDOUT)
        return True
    except subprocess.CalledProcessError:
        return False

def main():
    while True:  # Start an infinite loop
        user_input = input("Enter 'enable' to enable the actuator, 'disable' to disable it, or 'exit' to quit: ").strip().lower()

        if user_input == 'exit':
            print("Exiting the program.")
            break  # Exit the loop and finish the script

        is_running = is_launch_file_running()

        if user_input == 'enable':
            if not is_running:
                # If it's not running, kill everything, launch it, enable the controller,
                # kill everything again, and then launch 'joint_trajectory_controller.launch'
                stop_all_ros_nodes()
                subprocess.Popen(["roslaunch", "open_manipulator_controller", "open_manipulator_controller.launch"])
                time.sleep(5)  # Wait for the launch file to initiate
                set_actuator_state(True)  # Enable the actuator
                stop_all_ros_nodes()
                subprocess.Popen(["roslaunch", "move_to_pose_package", "joint_trajectory_controller.launch", "sim:=false"])
            else:
                # If 'open_manipulator_controller.launch' is running and the command comes for enabling,
                # enable the controller, kill all nodes, and run 'joint_trajectory_controller.launch'
                set_actuator_state(True)  # Enable the actuator
                stop_all_ros_nodes()
                subprocess.Popen(["roslaunch", "move_to_pose_package", "joint_trajectory_controller.launch", "sim:=false"])

        elif user_input == 'disable':
            if is_running:
                # If it is running, just disable the actuator. No need to change the launch files.
                set_actuator_state(False)  # Disable the actuator
            else:
                # If it's not running, we might need to launch it before disabling the actuator.
                # This part depends on whether you want to start 'open_manipulator_controller.launch' or not.
                print("The open_manipulator_controller is not running. Nothing to disable.")

        else:
            print("Invalid input. Please enter 'enable', 'disable', or 'exit'.")

        print("Action has been performed. Ready for the next command.")

if __name__ == "__main__":
    main()

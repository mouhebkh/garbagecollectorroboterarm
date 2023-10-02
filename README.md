# Manipulating Open Manipulator X for Object Pick-and-Place Operations

## Introduction:
This document provides guidelines for operating the Open Manipulator X robotic arm in a ROS environment for a pick-and-place project. The project aims to integrate YOLO for object detection and MoveIt! for robotic arm manipulation to autonomously pick identified objects and place them in designated locations. This document will guide you through the steps needed to run the robot with both GUI and automated movements, simulate operations using MoveIt! and RViz, and finally, operate the robot automatically to specific coordinates.
Prerequisites:
- Clone the GitHub repository into your workspace.
- Build the workspace and source ROS and the workspace before executing any commands.

# 1. Running the GUI
To initiate the graphical user interface (GUI) for controlling the Open Manipulator X, execute the following commands in separate terminals:
roslaunch open_manipulator_controller open_manipulator_controller.launch
roslaunch open_manipulator_control_gui [press tab to auto-complete]

# 2. Operating the Robot with MoveIt! and RViz
### 2.1 With Simulation
Execute the following command to operate the robot using MoveIt! and RViz with simulation:
roslaunch open_manipulator_controllers joint_trajectory_controller.launch
### 2.2 Without Simulation
To operate the robot without simulation, run:
roslaunch open_manipulator_controllers joint_trajectory_controller.launch sim:=false

# 3. Moving the Robot to Specific Coordinates
### 3.1 With Simulation
To automate the robot movement to specific coordinates with simulation, execute:
roslaunch move_to_pose_package joint_trajectory_controller.launch
### 3.2 Without Simulation
For operations without simulation, run:
roslaunch move_to_pose_package joint_trajectory_controller.launch sim:=false
Customizing Coordinates and Joint Angles
- Modify the valid coordinates in the `move_to_pose_package` located in the `scripts` folder. The filename is `move_to_pose.py`.
- To run the robot using code and customize the joint angles, edit the file in the `move_to_pose_package` within the `script` folder. 
The filename is `move_to_pose_coordinate_working.py`.

# Execution
To run the customized file, use the command, remember to run the joint trajectory controller launch file located in move to pose package before using this command:
rosrun move_to_pose_package move_to_pose_coordinate_working.py


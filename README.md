# Air Traffic Control: Balloon Catching Manipulator Arm
This repository contains software developed for ME 495 Embedded Systems in Robotics course. Written and maintained by Dilan Wijesinghe, Rintaroh Shima, Muye Jia, Hanbing Wu, and Ayush Gaggar. 

## Overview
The objective of this project is to play the game 'Don't let the balloon drop to the ground' with a Franka Emika Panda arm. It does this by tracking a falling balloon in real time, predicting where the balloon will land, moving to the location before the balloon lands, and pushing it up. This process is done semi-autonomously, with some human-robot interaction, and utilizes open-source computer vision and motion planning software packages.

![catch1](https://github.com/Dilan-Wijesinghe/AirTrafficControl/blob/main/media/IMG_7415.gif)

Our project is split into three packages:
### Tracking 
This package contains one node:
* `im_sub` - This node does the entire computer vision aspect of this project using a RealSense camera. It uses background subtraction and color detection to track a moving red balloon. With this information, it publishes to a topic called `balloon_coords`, which represents the real coordinates of the balloon in the camera frame.

![tracking1](https://github.com/Dilan-Wijesinghe/AirTrafficControl/blob/main/media/balloon_tracking.gif)

### Motion Planning Interfaces
This package contains GetPose, MultiplePose, and Place, which are srv files used in our custom `MoveIt` package.
### Motion Planning
Ths package contains two nodes:
* `simple_move` - This node is an implementation of the `MoveIt2` package in Python. It allows the user to move the Frank Emika Panda robot with trajectory and execution functions.
* `hit` - This node is responsible for gathering information from the `im_sub` node, making a prediction and sending a goal request to the Franka Emika Panda robot. 

## Hardware Requirements
This project requires the following hardware components:
* RealSense camera (information on connecting the RealSense can be found [here](https://nu-msr.github.io/ros_notes/ros2/realsense.html))
* Franka Emika Panda Robot (information on connecting to the Franka can be found [here](https://nu-msr.github.io/ros_notes/ros2/franka.html))
* Large paddle or equivalent
* Balloons

## Quickstart
1. After connecting with Franka, run `In station: ros2 launch franka_moveit_config moveit.launch.py use_rviz:=false robot_ip:=panda0.robot` in station.
2. Use the command `ros2 launch franka_moveit_config rviz.launch.py robot_ip:=panda0.robot` to launch the node with RViz.
3. Use the command `ros2 launch tracking tracking.launch.py` to launch im_sub.
4. Make sure the red balloon is currently in the frame of the camera, but 1.2m above the robot. 
5. Use the command `ros2 launch motion_planning move_and_hit.py` to launch simple_move and hit.
6. Type `ros2 service call /set_start geometry_msgs/msg/Pose "{position: {x: 0.4, y: 0.0, z: 0.25}, orientation: {x: 1., y: 0., z: 0., w: 0.}}"` to move the robot into the new home configuration.

## Future Works
For future works, our team would like to research faster and more reliable prediction methods. The Extended Kalman Filter comes to mind when thinking of an example. We would also like to look into movement planning outside of the `MoveIt` package, one that would allow us to directly communicate with the robot joints. This would allow us to significantly speed up the reaction time of the robot in response to the falling balloon. We were also interested in looking into multiple balloons of different colors and shapes, which would really challenge our computer vision side of things.
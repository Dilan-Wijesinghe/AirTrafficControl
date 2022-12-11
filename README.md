# ME 495 Embedded Systems Final Project
Authors: Ayush Gaggar, Dilan Wijesinghe, Muye Jia, Rintaroh Shima, Hanbing Wu

The `motion_planning` package will allow the user to input final end effector position and orientation,
then plan the trajectory; the user can choose to inspect the scene after trajectory planning, and then
execute the trajectory. Additionally, user can dynamically add box object to the scene at user-defined position.

The `motion_planning_interfaces` package defines service files that can be used in the `motion_planning` package.

## Quickstart
1. After connecting with franka, run `In station: ros2 launch franka_moveit_config moveit.launch.py use_rviz:=false robot_ip:=panda0.robot` in station
2. Use the command `ros2 launch franka_moveit_config rviz.launch.py robot_ip:=panda0.robot` to launch the node with rviz
3. Make sure the red balloon is currently in the frame of the camera, but 1.2 m above the robot. 
4. Use the command `ros2 launch motion_planning move_and_hit.py` to launch simple_move and hit.
5. Type `ros2 service call /set_pose `
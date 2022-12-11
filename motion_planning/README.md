# Motion Planning Package for AirTraffic Control
The package provides control schemes that can move the Franka arm to hit the balloon thrown by human players.

## Simple Move Node
This node does the path planning (screw and cartesian trajectory) for the Franka arm, and moves it to predicted pose to hit the balloon

2. After connecting with franka, run `In station: ros2 launch franka_moveit_config moveit.launch.py use_rviz:=false robot_ip:=panda0.robot` in station
3. Use the command `ros2 launch franka_moveit_config rviz.launch.py robot_ip:=panda0.robot` to launch the node with rviz
4. Use the command `ros2 run motion_planning simple_move` to launch simple_move

## Hit Node
This node does the prediction of balloon trajectory. It will publish a predicted hit pose to which the Franka arm will move

1. Use the command `ros2 run motion_planning hit` to run the Hit node
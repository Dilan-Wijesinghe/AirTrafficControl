# ME 495 Embedded Systems Final Project
Authors: Ayush Gaggar, Dilan Wijesinghe, Muye Jia, Rintaroh Shima, Hanbing Wu

`ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=false`

The `motion_planning` package will allow the user to input final end effector position and orientation,
then plan the trajectory; the user can choose to inspect the scene after trajectory planning, and then
execute the trajectory. Additionally, user can dynamically add box object to the scene at user-defined position.

The `motion_planning_interfaces` package defines service files that can be used in the `motion_planning` package.

## Quickstart
1. Use the command `ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true`.

2. Run `ros2 launch motion_planning simple_move.launch.py` to start the simple_move node.

3. To input a user-defined final end effector position, run `ros2 service call /set_pos motion_planning_interfaces/srv/GetPose "pose: {position: {x: ${x_pos}, y: ${y_pos}, z: ${z_pos}}}"` in a separate (sourced) terminal.

4. To input a user-defined final end effector orientation, run `ros2 service call /set_orient motion_planning_interfaces/srv/GetPose "pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: ${x_value}, y: ${y_value}, z: ${z_value}, w: ${w_value}}}"` in a separate (sourced) terminal.

5. To add a box object to the planning scene, run `ros2 service call /set_box_position motion_planning_interfaces/srv/GetPose "pose: {position: {x: ${x_pos}, y: ${y_pos}, z: ${z_pos}}, orientation: {x: ${x_value}, y: ${y_value}, z: ${z_value}, w: ${w_value}}}"` to set the position of the box, and the box will be added accordingly.

6. To wait to execute after planning, run `ros2 service call /wait_before_execute std_srvs/srv/SetBool "data: true"` (note that true would cause it to plan and execute, false would have it just plan). Finally, call the same service call from part 3, i.e. `ros2 service call /set_pos motion_planning_interfaces/srv/GetPose "pose: {position: {x: ${x_pos}, y: ${y_pos}, z: ${z_pos}}}"`.


## Adding a "balloon" into rviz
1. Make sure that the franka robot is already running (see Quickstart #1) 

2. Use the command `ros2 run motion_planning arena_node`. Note that the balloon's position is hardcoded (for the moment).
3. Run `ros2 service call /balloon_drop std_srvs/srv/Empty` to drop the balloon.
4. You need to kill the arena_node and rerun it to test again.

## Tracking balloon functionality
1. Make sure franka is running, and arena is also running
2. Use the command `ros2 run motion_planning balloon_move` to run the move_node with balloon tracking
3. Run `ros2 service call /balloon_drop std_srvs/srv/Empty` to drop the balloon.
4. The terminal should print the balloon's position as it falls

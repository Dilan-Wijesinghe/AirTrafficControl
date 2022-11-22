# Motion Planning Quickstart

## Using balloon_move to move robot_arm
1. Use the command `ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true`.
2. Use the command `ros2 run motion_planning balloon_move` to launch the move node.
3. Use the command `ros2 service call /set_orient motion_planning_interfaces/srv/GetPose "pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0., y: 0., z: 0., w: 1.}}"`. This will change the end-effector position to the set quaternion position
    Note that this quaternion can be changed, so long as the ee is level with some plane. Whatever quaternion position is chosen, make sure to change the hardcoded variable "self.orient_cons" to make sure trajectory planning is executed appropriately.
4. Now, using the command `ros2 service call /set_pos motion_planning_interfaces/srv/GetPose "pose: {position: {x: ${x_pos}, y: ${y_pos}, z: ${z_pos}}}"` in a separate (sourced) terminal, the e-e of the robot can be moved as per the user's inputs.
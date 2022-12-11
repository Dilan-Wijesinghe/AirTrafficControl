# Motion Planning Quickstart

## One and done guide to using simple_move
1. After connecting with franka, run `In station: ros2 launch franka_moveit_config moveit.launch.py use_rviz:=false robot_ip:=panda0.robot` in station
2. Use the command `ros2 launch franka_moveit_config rviz.launch.py robot_ip:=panda0.robot` to launch the node with rviz
3. Use the command `ros2 run motion_planning simple_move` to launch simple_move
4. Type `ros2 service call /`

## Using balloon_move to move robot_arm
1. Use the command `ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true`.
2. Use the command `ros2 run motion_planning balloon_move` to launch the move node.
3. Use the command `ros2 service call /set_orient motion_planning_interfaces/srv/GetPose "pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0., y: 0., z: 0., w: 1.}}"`. This will change the end-effector position to the set quaternion position
    Note that this quaternion can be changed, so long as the ee is level with some plane. Whatever quaternion position is chosen, make sure to change the hardcoded variable "self.orient_cons" to make sure trajectory planning is executed appropriately.
4. Now, using the command `ros2 service call /set_pos motion_planning_interfaces/srv/GetPose "pose: {position: {x: ${x_pos}, y: ${y_pos}, z: ${z_pos}}}"` in a separate (sourced) terminal, the e-e of the robot can be moved as per the user's inputs.

# ALTERNATIVELY:
4. Use the command `ros2 run motion_planning arena_node` to launch the balloon_marker. Note that the balloon's position is hardcoded (for the moment).
5. Run `ros2 service call /balloon_drop std_srvs/srv/Empty` to drop the balloon.
6. You need to kill the arena_node and rerun it to test again.

## Testing Cartesian move
1. Use the command `ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true`.
2. Use the command `ros2 run motion_planning simple_move` to launch the simple_move node.
3. Use the command `ros2 service call /set_orient motion_planning_interfaces/srv/GetPose "pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0., y: 0., z: 0., w: 1.}}"`. This will change the end-effector position to the set quaternion position
    Note that this quaternion can be changed, so long as the ee is level with some plane. Whatever quaternion position is chosen, make sure to change the hardcoded variable "self.orient_cons" to make sure trajectory planning is executed appropriately.
4. To publish one message `ros2 topic pub -1 /cartesian_waypoint geometry_msgs/msg/Pose "{position: {x: 0.4, y: 0.6, z: 0.4}, orientation: {x: 0., y: 0., z: 1., w: 0.}}"`
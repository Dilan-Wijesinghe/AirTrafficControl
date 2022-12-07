# Air Traffic Control - Tracking 

This package implements the computer vision part of the final project. It tracks moving 
objects within a scene and utilizes several OpenCV techniques to do so. This package 
uses Background Subtraction to find contours of a moving objects within the frame.
It then finds the centroid of these objects and calculates the real coordinates.
Finally it publishes these coordinates to a `balloon_coords` topic. 

Make sure that you are already running the Franka Rviz node - `ros2 launch franka_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true`

To run the balloon tracking - `ros2 launch tracking tracking.launch.py`

To run simple move - `ros2 run motion_planing simple_move`

To run hit - `ros2 run motion_planning hit`

## Quickstart
1. Use `ros2 launch realsense2_camera rs_launch.py 
        depth_module.profile:=1280x720x3 align_depth.enable:=true` 
        to start the realsense camera. 
2. Use `ros2 run tracking im_sub` to run the node in order to see the object tracking
3. Utilize `ros2 topic echo balloon_coords` to see the `x,y,z` coordinates of the moving object
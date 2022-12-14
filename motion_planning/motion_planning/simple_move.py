"""
This node will allow the user to input final end effector position and orientation.

The user can then plan the trajectory. He or she can choose to inspect the scene after trajectory
planning, and then execute the trajectory. Additionally, user can dynamically add box object to the
scene at user-defined position.

PUBLISHER:
    + /planning_scene (moveit_msgs/msg/PlanningScene) - Publish planning scene
    + /curr_ee_pos (geometry_msgs/msg/Point) - Publish current end-effector position
    + /cart_cycle_complete (std_msgs/msg/Empty) - Publish Empty object to act as state change
                                                  signal

SUBSCRIBER:
    + /joint_states (sensor_msgs/msg/JointState) - Subscribe to the joint states of the robot
    + /set_pose (geometry_msgs/msg/Pose) - Get the end effector position and
                                           orientation
    + /set_pos (geometry_msgs/msg/Pose) - Get the end effector position
    + /set_orient (motion_planning_interfaces/srv/GetPose) - Get end effector orientation
    + /cartesian_waypoint (geometry_msgs/msg/PoseArray) - Get waypoints for cartesian path
    + /panda_arm_controller/state (sensor_msgs/msg/JointState) - Get current Franka arm joint
                                                                 angles
    + /set_start (geometry_msgs/msg/Pose) - Get the starting Franka arm pose

SERVICES:
    + /set_box_position (motion_planning_interfaces/srv/GetPose) - Set box position in the
                                                                   planning scene
    + /set_orient (motion_planning_interfaces/srv/GetPose) - Set the orientation
    + /wait_before_execute (std_srvs/srv/SetBool) - Wait to execute after planning
    + /set_start (motion_planning_interfaces/srv/GetPose) - Set up the start point of the end
                                                            effector

ACTION_CLIENT:
    + /move_action (moveit_msgs/action/MoveGroup) - Send MotionPlanRequest for screw trajectory
    + /execute_trajectory (moveit_msgs/action/ExecuteTrajectory) - Execute planned trajectory

CLIENT:
    + /compute_ik (moveit_msgs/srv/GetPositionIK) - Compute inverse kinematics for designated
                                                    end-effector pose
    + /get_planning_scene (moveit_msgs/srv/GetPlanningScene) - Get planning scene msg
"""

import rclpy
from enum import Enum, auto
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, \
    WorkspaceParameters, RobotState, RobotTrajectory, PlanningOptions, PositionIKRequest, \
    PlanningScene, PlanningSceneWorld, CollisionObject, PlanningSceneComponents, \
    OrientationConstraint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPlanningScene, GetCartesianPath
from control_msgs.msg import JointTrajectoryControllerState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, PoseStamped, Pose, Point, Quaternion, PoseArray
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from std_srvs.srv import SetBool as Bool
from std_msgs.msg import Empty
from tf2_ros import TransformException
from motion_planning_interfaces.srv import GetPose


class State(Enum):
    """
    Current state of the robot.

    Determine whether the desired position has been set.
    """

    NOTSET = auto()
    GO = auto()
    OBTAINED = auto()


class Mover(Node):

    def __init__(self):
        """Initialize variables, publisher, subscriber, services and a timer."""
        super().__init__('simple_move')

        timer_period = 0.01
        self.cbgroup = MutuallyExclusiveCallbackGroup()
        self.create_timer(timer_period, self.timer_callback)

        # Action clients for traj execution and IK
        self.send_pos = ActionClient(self, MoveGroup, 'move_action')
        self.execute_traj = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik',
                                            callback_group=self.cbgroup)

        # Add box functionality
        self.add_box_cli = self.create_client(GetPlanningScene, '/get_planning_scene',
                                              callback_group=self.cbgroup)
        self.box_pos = self.create_service(GetPose, '/set_box_position', self.set_box)
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 1)
        self.origin = Pose()
        self.origin.position = Point(x=0.0, y=0.0, z=0.0)
        self.box_position = Point()
        self.state = State.NOTSET
        self.scene_state = State.NOTSET
        self.scene_result = PlanningScene()

        # For planning trajectory
        self.get_cartesian_waypoint = self.create_subscription(
                                                               PoseArray, 'cartesian_waypoint',
                                                               self.cartesian_waypoint_callback,
                                                               10)
        self.get_cartesian_traj = self.create_client(GetCartesianPath, 'compute_cartesian_path',
                                                     callback_group=self.cbgroup)
        self.get_position = self.create_subscription(Pose, 'set_pos',
                                                     self.get_position_callback, 10)
        self.set_ori = self.create_service(GetPose, 'set_orient', self.get_orient_callback)
        self.get_pose = self.create_subscription(Pose, 'set_pose', self.get_pose_callback, 10)
        self.wait_before_execute_service = self.create_service(Bool, "wait_before_execute",
                                                               self.wait_callback)
        # TF buffer for our listener
        self.tf_buffer = Buffer()

        # TL to get ee pos
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.joint_state_sub = self.create_subscription(JointState, "panda_arm_controller/state",
                                                        self.js_callback, 10)
        self.set_start = self.create_subscription(Pose, '/set_start', self.set_start_callback, 10)
        self.curr_pos_pub = self.create_publisher(Point, 'curr_ee_pos', 10)
        self.state_pub = self.create_publisher(Empty, 'cart_cycle_complete', 10)

        # State variables for control
        self.curr_pos = Point()
        self.execute_immediately = True
        self.robot_state = State.NOTSET
        self.set_start_state = State.NOTSET
        self.user_start_config = False
        self.cartesian = State.NOTSET
        self.send_cart_goal = State.NOTSET
        self.reach_waypoint = State.NOTSET
        self.gohome = State.NOTSET
        self.waypoint_indx = 0

        # Useful variables
        # Initial starting joint_states, UNLESS user has specified a start configuration
        self.start_js = [0.0,
                         -0.7853981633974483,
                         0.0,
                         -2.356194490192345,
                         0.0,
                         1.5707963267948966,
                         0.7853981633974483,
                         0.035,
                         0.035,
                         ]
        self.start_config = Pose()
        self.cartesian_waypoint = []
        self.cart_start = Pose()
        self.cart_traj = RobotTrajectory()
        self.final_js = []
        self.planned_trajectory = RobotTrajectory()
        self.traj_for_later = RobotTrajectory()
        self.ik_states = PositionIKRequest()
        self.ik_robot_states = PoseStamped()
        self.ik_pose = Pose()
        self.ik_soln = RobotState()
        self.max_vel_scale = 0.1
        self.max_acc_scale = 0.1
        self.hard_code_vel_scale = 1.0
        self.balloon_z = 0.12  # m from paddle to balloon
        self.orient_constraint = Quaternion(x=0., y=0., z=0., w=1.)
        self.joint_tolerance = 0.3 / 10.0  # We need to change this

        # m from paddle to balloon
        self.balloon_z = 0.12
        self.orient_constraint = Quaternion(x=1., y=0., z=0., w=0.)
        self.joint_tolerance = 0.1 / 20.0
        self.tofu = 0
        self.joint_traj_controller_state = JointTrajectoryControllerState()
        self.curr_joint_pos = []
        self.empty_msg = Empty()

    def cartesian_waypoint_callback(self, waypoints: PoseArray):
        """
        Get waypoint(s) for cartesian path.

        Args: waypoints (geometry_msgs/msg/Pose): List of Pose for desired cartesian path.

        Returns: None
        """
        self.cartesian_waypoint = []
        for waypoint in range(len(waypoints.poses)):
            if self.tofu == -1:
                waypoints.poses[waypoint].position.z = self.curr_pos.z - 0.2
            else:
                waypoints.poses[waypoint].position.z = self.curr_pos.z
            self.cartesian_waypoint.append(waypoints.poses[waypoint])

        self.reach_waypoint = State.OBTAINED
        self.cartesian = State.GO
        self.gohome = State.NOTSET

    def set_start_callback(self, request):
        """
        Get the start position and change state.

        Args: request (GetPoseRequest): Position and orientation of the end effector

        Returns: None
        """
        self.get_logger().info("Set starting pose!")
        if self.set_start_state == State.NOTSET:
            self.start_config.position.x = request.pose.position.x
            self.start_config.position.y = request.pose.position.y
            self.start_config.position.z = request.pose.position.z
            self.start_config.orientation.x = request.pose.orientation.x
            self.start_config.orientation.y = request.pose.orientation.y
            self.start_config.orientation.z = request.pose.orientation.z
            self.start_config.orientation.w = request.pose.orientation.w

            self.set_start_state = State.GO
            self.user_start_config = True
            self.execute_immediately = False

    def set_box(self, request, response):
        """
        Set box pose.

        Args: request (GetPoseRequest): Position and orientation of the box
              response (GetPoseResponse): the response object

        Returns: A GetPoseResponse
        """
        self.get_logger().info("Starting Set Box")
        self.box_position.x = request.pose.position.x
        self.box_position.y = request.pose.position.y
        self.box_position.z = request.pose.position.z

        if self.state == State.NOTSET:
            self.state = State.GO

        return response

    def get_position_callback(self, request):
        """
        Get the position.

        Args: request (GetPoseRequest): Position and orientation of the end effector
              response (GetPoseResponse): the response object

        Returns: A GetPoseResponse
        """
        self.get_logger().info("Starting Set Position")
        self.ik_pose.position.x = request.pose.position.x
        self.ik_pose.position.y = request.pose.position.y
        self.ik_pose.position.z = request.pose.position.z

        if self.robot_state == State.NOTSET:
            self.robot_state = State.GO

    def get_orient_callback(self, request, response):
        """
        Get the orientation of the end effector.

        Args: request (GetPoseRequest): Position and orientation of the box
              response (GetPoseResponse): the response object

        Returns: A GetPoseResponse
        """
        self.get_logger().info("Starting Set Orient")
        self.ik_pose.position.x = self.curr_pos.x
        self.ik_pose.position.y = self.curr_pos.y
        self.ik_pose.position.z = self.curr_pos.z
        self.ik_pose.orientation.x = request.pose.orientation.x
        self.ik_pose.orientation.y = request.pose.orientation.y
        self.ik_pose.orientation.z = request.pose.orientation.z
        self.ik_pose.orientation.w = request.pose.orientation.w

        if self.robot_state == State.NOTSET:
            self.robot_state = State.GO

        return response

    def get_pose_callback(self, request):
        """
        Get the position and orientation of the end effector.

        Args: request (GetPoseRequest): Position and orientation of the box

        Returns: None
        """
        self.get_logger().info("Pose received!")
        self.ik_pose.position.x = request.position.x
        self.ik_pose.position.y = request.position.y
        self.ik_pose.position.z = request.position.z
        self.ik_pose.orientation.x = request.orientation.x
        self.ik_pose.orientation.y = request.orientation.y
        self.ik_pose.orientation.z = request.orientation.z
        self.ik_pose.orientation.w = request.orientation.w

        if self.robot_state == State.NOTSET:
            self.robot_state = State.GO

    async def timer_callback(self):
        """
        Determine the state of the robot.

        Args: None

        Returns: None
        """
        # Check whether the EE reaches the waypoint
        if self.reach_waypoint == State.OBTAINED and len(self.cartesian_waypoint) != 0:
            # Use the transform listener to determine curr EE pose
            # This is a position object
            target = self.cartesian_waypoint[self.waypoint_indx]
            targetx = target.position.x
            targety = target.position.y

            if abs(self.curr_pos.x - targetx) < 0.3 and abs(self.curr_pos.y - targety) < 0.3:
                if self.tofu < 50:
                    self.tofu += 1
                else:
                    self.tofu = 0
                    print(f'index: {self.waypoint_indx}')
                    self.waypoint_indx += 1

                    # Only send the cartesian request if the index is in range
                    if self.waypoint_indx < len(self.cartesian_waypoint):
                        self.cartesian_waypoint[self.waypoint_indx].position.z = \
                                                self.cartesian_waypoint[self.waypoint_indx-1]\
                                                .position.z + 0.1
                        info_list = self.send_cartesian_goal()

                        cart_future = self.get_cartesian_traj\
                            .call_async(GetCartesianPath.Request(header=info_list[0],
                                        start_state=info_list[1],
                                        group_name=info_list[2],
                                        link_name=info_list[3],
                                        waypoints=info_list[4],
                                        max_step=info_list[5],
                                        jump_threshold=info_list[6],
                                        avoid_collisions=info_list[7]))

                        await cart_future

                        if cart_future.done():
                            if cart_future.result().error_code.val < -1:
                                self.get_logger().info('Cannot get cartesian path')
                                print('The number of waypoints executed were:')
                                print(cart_future.result().fraction)
                                self.send_cart_goal = State.NOTSET
                            else:
                                self.cart_traj = cart_future.result().solution
                                self.planned_trajectory = self.cart_traj
                                self.cartesian = State.NOTSET
                                self.exe_trajectory()
                                self.get_logger().info('Start 2nd Cartesian Path!')
                                print('cycle complete --> restart')
                                self.tofu = -1
                                self.cartesian_waypoint = []
                                self.state_pub.publish(Empty())
                                self.waypoint_indx = 0
                                self.reach_waypoint = State.NOTSET

        # Obtain joint states for start Pose
        if self.set_start_state == State.GO:

            # Filling in IK request msg
            self.ik_states.group_name = "panda_arm"

            # Final joint xyz position
            self.ik_pose.position = self.start_config.position

            # PoseStamped msg
            self.ik_robot_states.pose = self.start_config

            # Filling in robot state field in IK request
            self.ik_states.pose_stamped = self.ik_robot_states

            ik_future = self.ik_client.call_async(GetPositionIK.Request(ik_request=self.ik_states))
            await ik_future

            if ik_future.done() and self.set_start_state != State.OBTAINED:
                if ik_future.result().error_code.val < -1:
                    self.get_logger().info('The start pose is unexecutable')
                    self.robot_state = State.NOTSET
                else:
                    # The service returns RobotState object
                    self.start_js = ik_future.result().solution.joint_state.position
                    self.robot_state = State.NOTSET
                    self.set_start_state = State.NOTSET

        # Add box to the scene
        if self.state == State.GO:
            plan_component = PlanningSceneComponents()
            get_scene_future = self.add_box_cli.call_async(
                GetPlanningScene.Request(components=plan_component))
            await get_scene_future

            if get_scene_future.done() and self.scene_state != State.OBTAINED:
                self.scene_result = get_scene_future.result().scene
                self.scene_state = State.OBTAINED

            if self.scene_state == State.OBTAINED:
                # Fill in the desired part of scene msg
                collision_obj = CollisionObject()
                shape_msg = SolidPrimitive()
                plan_scene_world = PlanningSceneWorld()

                # Add box shape
                shape_msg.type = 1
                shape_msg.dimensions = [0.1, 0.1, 0.1]

                # Add origin to collision obj
                collision_obj.pose = self.origin

                # Add box msg to collision obj msg
                collision_obj.primitives = [shape_msg]
                box_pose = Pose()
                box_pose.position = self.box_position
                collision_obj.primitive_poses = [box_pose]

                # Add time stamp
                time = self.get_clock().now().to_msg()
                header_msg = Header()
                header_msg.stamp = time
                header_msg.frame_id = 'panda_link0'
                collision_obj.header = header_msg

                # Add collision obj into planning scene world msg
                plan_scene_world.collision_objects = [collision_obj]

                # Add planning scene into planning scene msg
                self.scene_result.world = plan_scene_world

                # Publish to planning scene topic
                self.scene_pub.publish(self.scene_result)

        if self.robot_state == State.NOTSET:
            try:
                self.tf_base_arm = self.tf_buffer.lookup_transform('panda_link0',
                                                                   'panda_hand', rclpy.time.Time())
                self.curr_pos.x = self.tf_base_arm.transform.translation.x
                self.curr_pos.y = self.tf_base_arm.transform.translation.y
                self.curr_pos.z = self.tf_base_arm.transform.translation.z
                self.curr_pos_pub.publish(self.curr_pos)
            except TransformException as ex:
                self.get_logger().info(f'Could not transform : {ex}')

        # Obtain final joint state for target Pose
        if self.robot_state == State.GO:
            # Filling in IK request msg
            self.ik_states.group_name = "panda_arm"

            # PoseStamped msg
            self.ik_robot_states.pose = self.ik_pose

            # Filling in robot state field in IK request
            self.ik_states.pose_stamped = self.ik_robot_states
            self.ik_states.avoid_collisions = True
            js = JointState()
            js_header = Header()
            js_header.stamp = self.get_clock().now().to_msg()
            js_header.frame_id = 'panda_link0'
            js.header = js_header
            js.name = [
                'panda_joint1',
                'panda_joint2',
                'panda_joint3',
                'panda_joint4',
                'panda_joint5',
                'panda_joint6',
                'panda_joint7',
                'panda_finger_joint1',
                'panda_finger_joint2'
            ]
            js.position = self.curr_joint_pos
            self.ik_states.robot_state.joint_state = js
            ik_future = self.ik_client.call_async(GetPositionIK.Request(ik_request=self.ik_states))
            await ik_future

            if ik_future.done() and self.robot_state != State.OBTAINED:
                if ik_future.result().error_code.val < -1:
                    self.get_logger().info('The planned trajectory is unexecutable')
                    self.robot_state = State.NOTSET
                else:
                    self.final_js = ik_future.result().solution.joint_state.position
                    self.robot_state = State.NOTSET
                    self.get_logger().info("Sending Goal")
                    self.send_goal()

        # Obtain final joint state for cartesian waypoint(s)
        if self.cartesian == State.GO:
            self.get_logger().info("Doing Cartesian")
            info_list = self.send_cartesian_goal()
            cart_future = self.get_cartesian_traj.call_async(
                GetCartesianPath.Request(header=info_list[0],
                                         start_state=info_list[1],
                                         group_name=info_list[2],
                                         link_name=info_list[3],
                                         waypoints=info_list[4],
                                         max_step=info_list[5],
                                         jump_threshold=info_list[6],
                                         avoid_collisions=info_list[7]))
            await cart_future

            if cart_future.done():
                if cart_future.result().error_code.val < -1:
                    self.get_logger().info('Cannot get cartesian path')
                    print('The number of waypoints executed were:')
                    self.send_cart_goal = State.NOTSET
                else:
                    self.cart_traj = cart_future.result().solution
                    self.planned_trajectory = self.cart_traj
                    self.cartesian = State.NOTSET
                    self.exe_trajectory()
                    self.get_logger().info('Start executing Cartesian Path!')

    def send_cartesian_goal(self):
        """
        Send the request msg to obtain Cartesian trajectory.

        Args: None

        Returns: List of info.
        """
        header_msg = Header()
        header_msg.stamp = self.get_clock().now().to_msg()
        header_msg.frame_id = 'panda_link0'

        # Start of cartesian path
        cart_rob_state = RobotState()
        cart_start_js = JointState()

        cart_start_js.name = [
            'panda_joint1',
            'panda_joint2',
            'panda_joint3',
            'panda_joint4',
            'panda_joint5',
            'panda_joint6',
            'panda_joint7',
            'panda_finger_joint1',
            'panda_finger_joint2'
        ]
        cart_start_js.position = self.curr_joint_pos
        cart_start_js.header = header_msg
        cart_rob_state.joint_state = cart_start_js

        group_name = "panda_arm"

        # Link for which cartesian path is computed
        link_name = 'panda_hand_tcp'

        # Waypoints list
        print(f"Curr Length of Cart WP: {len(self.cartesian_waypoint)} idx: {self.waypoint_indx}")
        way_pts = [self.cartesian_waypoint[self.waypoint_indx]]

        if self.waypoint_indx == 0:
            max_step = 0.1
        else:
            max_step = 0.01
        jump_threshold = 50.
        avoid_coll = True

        cons = Constraints()
        cons.name = 'stay_level'
        orient_cons = OrientationConstraint()
        orient_cons.header.stamp = self.get_clock().now().to_msg()
        orient_cons.header.frame_id = 'panda_hand'
        orient_cons.orientation = self.orient_constraint
        orient_cons.link_name = 'panda_hand_tcp'
        orient_cons.absolute_x_axis_tolerance = 0.01
        orient_cons.absolute_y_axis_tolerance = 0.01
        orient_cons.absolute_z_axis_tolerance = 0.01
        orient_cons.weight = 1.0
        cons.orientation_constraints.append(orient_cons)

        # Cons if want constraint
        info_list = [header_msg, cart_rob_state, group_name,
                     link_name, way_pts, max_step, jump_threshold, avoid_coll]

        return info_list

    def send_goal(self):
        """
        Send the request msg to obtain non-cartesian trajectory.

        Args: None

        Returns: None
        """
        move_group_goal = MoveGroup.Goal()
        goal_msg = MotionPlanRequest()

        # Change to PlanOnly
        plan_option = PlanningOptions()
        plan_option.plan_only = True
        move_group_goal.planning_options = plan_option

        # Add time stamp
        time = self.get_clock().now().to_msg()
        work_msg = WorkspaceParameters()
        header_msg = Header()
        header_msg.stamp = time
        header_msg.frame_id = 'panda_link0'
        work_msg.header = header_msg

        # Add workspace space limits
        min_corner = Vector3()
        min_corner.x = -1.0
        min_corner.y = -1.0
        min_corner.z = -1.0

        max_corner = Vector3()
        max_corner.x = 1.0
        max_corner.y = 1.0
        max_corner.z = 1.0

        work_msg.min_corner = min_corner
        work_msg.max_corner = max_corner
        goal_msg.workspace_parameters = work_msg

        # Input desired starting position
        start_pos = RobotState()
        js = JointState()
        js_header = Header()
        js_header.stamp = self.get_clock().now().to_msg()
        js_header.frame_id = 'panda_link0'
        js.header = js_header

        js_name = [
            'panda_joint1',
            'panda_joint2',
            'panda_joint3',
            'panda_joint4',
            'panda_joint5',
            'panda_joint6',
            'panda_joint7',
            'panda_finger_joint1',
            'panda_finger_joint2'
        ]
        js.name = js_name
        js.position = self.curr_joint_pos

        start_pos.joint_state = js
        goal_msg.start_state = start_pos

        joint_constraints_list = Constraints()
        jc_list = []

        joint1 = JointConstraint()
        joint1.joint_name = "panda_joint1"
        joint1.position = self.final_js[0]
        joint1.tolerance_above = 0.0001
        joint1.tolerance_below = 0.0001
        joint1.weight = 1.0
        jc_list.append(joint1)

        joint2 = JointConstraint()
        joint2.joint_name = "panda_joint2"
        joint2.position = self.final_js[1]
        joint2.tolerance_above = 0.0001
        joint2.tolerance_below = 0.0001
        joint2.weight = 1.0
        jc_list.append(joint2)

        joint3 = JointConstraint()
        joint3.joint_name = "panda_joint3"
        joint3.position = self.final_js[2]
        joint3.tolerance_above = 0.0001
        joint3.tolerance_below = 0.0001
        joint3.weight = 1.0
        jc_list.append(joint3)

        joint4 = JointConstraint()
        joint4.joint_name = "panda_joint4"
        joint4.position = self.final_js[3]
        joint4.tolerance_above = 0.0001
        joint4.tolerance_below = 0.0001
        joint4.weight = 1.0
        jc_list.append(joint4)

        joint5 = JointConstraint()
        joint5.joint_name = "panda_joint5"
        joint5.position = self.final_js[4]
        joint5.tolerance_above = 0.0001
        joint5.tolerance_below = 0.0001
        joint5.weight = 1.0
        jc_list.append(joint5)

        joint6 = JointConstraint()
        joint6.joint_name = "panda_joint6"
        joint6.position = self.final_js[5]
        joint6.tolerance_above = 0.0001
        joint6.tolerance_below = 0.0001
        joint6.weight = 1.0
        jc_list.append(joint6)

        joint7 = JointConstraint()
        joint7.joint_name = "panda_joint7"
        joint7.position = self.final_js[6]
        joint7.tolerance_above = 0.0001
        joint7.tolerance_below = 0.0001
        joint7.weight = 1.0
        jc_list.append(joint7)

        joint_constraints_list.joint_constraints = jc_list
        goal_msg.goal_constraints = [joint_constraints_list]

        goal_msg.pipeline_id = 'move_group'
        goal_msg.group_name = 'panda_manipulator'
        goal_msg.num_planning_attempts = 10
        goal_msg.allowed_planning_time = 5.0
        goal_msg.max_velocity_scaling_factor = self.max_vel_scale
        goal_msg.max_acceleration_scaling_factor = self.max_acc_scale
        move_group_goal.request = goal_msg

        # Send the goal request
        self.send_pos.wait_for_server()
        print(move_group_goal.request.max_velocity_scaling_factor)
        print(move_group_goal.request.max_cartesian_speed)
        self.send_goal_future = self.send_pos.send_goal_async(move_group_goal)
        self.send_goal_future.add_done_callback(self.move_cli_callback)

    def move_cli_callback(self, future):
        """
        Get a future that will complete when the result is ready.

        Args: future: A future object

        Returns: None
        """
        result = future.result()
        self._get_result_future = result.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Get the result.

        Args: future: A future object

        Returns: None
        """
        result = future.result().result
        self.planned_trajectory = result.planned_trajectory

        # Execute the trajectory after getting the result, if self.execute_immediately is true
        if self.execute_immediately is True:
            self.get_logger().info("Start executing trajectory IMMEDIATELY")
            self.exe_trajectory()
            return
        else:
            self.robot_state = State.NOTSET

    def exe_trajectory(self):
        """
        Execute the trajectory.

        Args: None

        Returns: None
        """
        trajectory = ExecuteTrajectory.Goal()
        self.get_logger().info("Execute trajectory yo")
        for i in range(len(self.planned_trajectory
                       .joint_trajectory.points)):
            for j in range(len(self.planned_trajectory.joint_trajectory.points[i]
                           .velocities)):
                self.planned_trajectory.joint_trajectory.points[i].velocities[j] \
                             = self.planned_trajectory.joint_trajectory.points[i] \
                             .velocities[j]*self.hard_code_vel_scale
                self.planned_trajectory.joint_trajectory.points[i].accelerations[j] \
                    = self.planned_trajectory.joint_trajectory.points[i]\
                    .accelerations[j]*self.hard_code_vel_scale
            new_time = (self.planned_trajectory.joint_trajectory.points[i].
                        time_from_start.sec + self.planned_trajectory.joint_trajectory.
                        points[i].time_from_start.nanosec * 1e-9)/self.hard_code_vel_scale
            self.planned_trajectory.joint_trajectory.points[i].time_from_start.sec = int(new_time)
            self.planned_trajectory.joint_trajectory.points[i]\
                .time_from_start.nanosec = int((new_time % 1) * 1e9)

        traj_duration = (self.planned_trajectory.joint_trajectory.points[-1].time_from_start)
        self.traj_time = traj_duration.sec + traj_duration.nanosec * 10**(-9)
        print(f"Plan + Execute Time: {self.traj_time}")
        if self.traj_time < 0.1:
            print(self.planned_trajectory.joint_trajectory.points)
        trajectory.trajectory = self.planned_trajectory

        # Send the request
        self.execute_traj.wait_for_server()

        self.exe_future = self.execute_traj.send_goal_async(trajectory)
        self.exe_future.add_done_callback(self.exe_done_callback)

    def exe_done_callback(self, future):
        """
        Check if execution is done.

        Args: future: A future object

        Returns: None
        """
        self.get_logger().info("Trajectory execution done")
        self.robot_state = State.NOTSET
        self.get_logger().info(f"Current State is {self.robot_state}")

    def wait_callback(self, request, response):
        """
        Wait to execute trajectory.

        Args: request (BoolRequest): True or False (whether to execute immediately or not)
              response (BoolResponse): the response object

        Returns: A BoolResponse
        """
        self.execute_immediately = request.data
        return response

    def js_callback(self, msg):
        """
        Subscription to joint_states.

        Specifies the position as a float of the following:
            panda_joint1, panda_joint2, panda_joint3,
            panda_joint4, panda_joint5, panda_joint6,
            panda_joint7, panda_finger_joint1, panda_finger_joint2

        Args: msg (sensor_msgs/msg/JointState): The joint state of the robot

        Returns: None
        """
        self.joint_traj_controller_state = msg
        self.curr_joint_pos = self.joint_traj_controller_state.actual.positions
        return


def main(args=None):
    rclpy.init(args=None)
    node = Mover()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

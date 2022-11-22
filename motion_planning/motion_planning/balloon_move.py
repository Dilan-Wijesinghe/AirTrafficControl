"""
This node will allow the user to input final end effector position and orientation.

The user can then plan the trajectory. He or she can choose to inspect the scene after trajectory
planning, and then execute the trajectory. Additionally, user can dynamically add box object to the
scene at user-defined position.

PUBLISHER:
    + /planning_scene (PlanningScene) - Publish planning scene

SUBSCRIBER:
    + /joint_states (JointState) - Subscribe to the joint states of the robot

SERVICES:
    + /set_box_position (motion_planning_interfaces/srv/GetPose) - Set box position in the
                                                                   planning scene
    + /set_pos (motion_planning_interfaces/srv/GetPose) - Set the end effector position
    + /set_orient (motion_planning_interfaces/srv/GetPose) - Set the orientation
    + /set_pose (motion_planning_interfaces/srv/GetPose) - Set the end effector position and
                                                           orientation
    + /wait_before_execute (std_srvs/srv/SetBool) - Wait to execute after planning
    + /set_start (motion_planning_interfaces/srv/GetPose) - Set up the start point of the end
                                                            effector

"""

import rclpy
from enum import Enum, auto
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, \
    WorkspaceParameters, RobotState, RobotTrajectory, PlanningOptions, PositionIKRequest, \
    PlanningScene, PlanningSceneWorld, CollisionObject, PlanningSceneComponents
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPlanningScene
from trajectory_msgs.msg import JointTrajectory
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, PoseStamped, Pose, Point
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from std_srvs.srv import SetBool as Bool
from tf2_ros import TransformException
from visualization_msgs.msg import Marker
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

        # for execute_traj and IK
        self.send_pos = ActionClient(self, MoveGroup, 'move_action')
        self.execute_traj = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik',
                                            callback_group=self.cbgroup)

        # add box functionality
        self.add_box_cli = self.create_client(GetPlanningScene, '/get_planning_scene',
                                              callback_group=self.cbgroup)
        self.box_pos = self.create_service(GetPose, '/set_box_position', self.set_box)
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 1)
        self.balloon_sub = self.create_subscription(Marker, '/balloon_marker', self.balloon_callback, 10)
        self.origin = Pose()
        self.origin.position = Point(x=0.0, y=0.0, z=0.0)
        self.box_position = Point()
        self.balloon_position = Point()
        self.state = State.NOTSET
        self.scene_state = State.NOTSET
        self.scene_result = PlanningScene()

        # for planning trajectory
        self.set_position = self.create_service(GetPose, 'set_pos', self.set_position_callback)
        self.set_ori = self.create_service(GetPose, 'set_orient', self.get_orient_callback)
        self.set_pose = self.create_service(GetPose, 'set_pose', self.set_pose_callback)
        self.wait_before_execute_service = self.create_service(
            Bool, "wait_before_execute", self.wait_callback)
        self.tf_buffer = Buffer() # TF buffer for our listener
        self.tf_listener = TransformListener(self.tf_buffer, self) # TL to get ee pos
        self.joint_state_sub = self.create_subscription(JointState, "joint_states",
                                                        self.js_callback, 10)
        self.set_start = self.create_service(GetPose, '/set_start', self.set_start_callback)

        self.curr_pos = None
        self.execute_immediately = True

        self.robot_state = State.NOTSET
        self.set_start_state = State.NOTSET
        self.user_start_config = False
        # initial starting joint_states, UNLESS user has specified a start configuration
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
        self.final_js = []
        self.planned_trajectory = RobotTrajectory()
        self.traj_for_later = RobotTrajectory()
        self.ik_states = PositionIKRequest()
        self.ik_robot_states = PoseStamped()
        self.ik_pose = Pose()
        self.ik_soln = RobotState()

    def set_start_callback(self, request, response):
        """
        Get the start position and change state.

        Args: request (GetPoseRequest): Position and orientation of the end effector
              response (GetPoseResponse): the response object

        Returns: A GetPoseResponse
        """
        self.get_logger().info("Starting Set Start")
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

        return response

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

    def set_position_callback(self, request, response):
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

        return response

    def get_orient_callback(self, request, response):
        """
        Get the orientation of the end effector.

        Args: request (GetPoseRequest): Position and orientation of the box
              response (GetPoseResponse): the response object

        Returns: A GetPoseResponse
        """
        self.get_logger().info("Starting Set Orient")
        self.ik_pose.position.x = self.curr_pos[0]
        self.ik_pose.position.y = self.curr_pos[1]
        self.ik_pose.position.z = self.curr_pos[2]
        self.ik_pose.orientation.x = request.pose.orientation.x
        self.ik_pose.orientation.y = request.pose.orientation.y
        self.ik_pose.orientation.z = request.pose.orientation.z
        self.ik_pose.orientation.w = request.pose.orientation.w

        if self.robot_state == State.NOTSET:
            self.robot_state = State.GO

        return response

    def set_pose_callback(self, request, response):
        """
        Get the position and orientation of the end effector.

        Args: request (GetPoseRequest): Position and orientation of the box
              response (GetPoseResponse): the response object

        Returns: A GetPoseResponse
        """
        self.get_logger().info("Starting Set Pose")
        self.ik_pose.position.x = request.pose.position.x
        self.ik_pose.position.y = request.pose.position.y
        self.ik_pose.position.z = request.pose.position.z
        self.ik_pose.orientation.x = request.pose.orientation.x
        self.ik_pose.orientation.y = request.pose.orientation.y
        self.ik_pose.orientation.z = request.pose.orientation.z
        self.ik_pose.orientation.w = request.pose.orientation.w

        if self.robot_state == State.NOTSET:
            self.robot_state = State.GO

        return response

    async def timer_callback(self):
        """
        Determine the state of the robot.

        Args: None

        Returns: None
        """

        # The following IF is IK for (user) START CONFIG.
        if self.set_start_state == State.GO:
            # filling in IK request msg
            self.ik_states.group_name = "panda_arm"

            # final joint xyz position
            self.ik_pose.position = self.start_config.position

            # PoseStamped msg
            self.ik_robot_states.pose = self.start_config

            # filling in robot state field in IK request
            self.ik_states.pose_stamped = self.ik_robot_states

            ik_future = self.ik_client.call_async(GetPositionIK.Request(ik_request=self.ik_states))
            await ik_future

            if ik_future.done() and self.set_start_state != State.OBTAINED:
                # the service returns RobotState object
                self.start_js = ik_future.result().solution.joint_state.position
                self.robot_state = State.OBTAINED
                self.send_goal()
                # self.set_start_state = State.OBTAINED
        
        # for add box
        if self.state == State.GO:
            plan_component = PlanningSceneComponents()
            get_scene_future = self.add_box_cli.call_async(
                GetPlanningScene.Request(components=plan_component))
            await get_scene_future

            if get_scene_future.done() and self.scene_state != State.OBTAINED:
                self.scene_result = get_scene_future.result().scene
                self.scene_state = State.OBTAINED

            if self.scene_state == State.OBTAINED:
                # fill in the desired part of scene msg
                collision_obj = CollisionObject()
                shape_msg = SolidPrimitive()
                plan_scene_world = PlanningSceneWorld()

                # add box shape
                shape_msg.type = 1
                shape_msg.dimensions = [0.1, 0.1, 0.1]

                # add origin to collision obj
                collision_obj.pose = self.origin

                # add box msg to collision obj msg
                collision_obj.primitives = [shape_msg]
                box_pose = Pose()
                box_pose.position = self.box_position
                collision_obj.primitive_poses = [box_pose]

                # add time stamp
                time = self.get_clock().now().to_msg()
                header_msg = Header()
                header_msg.stamp = time
                header_msg.frame_id = 'panda_link0'
                collision_obj.header = header_msg

                # add collision obj into planning scene world msg
                plan_scene_world.collision_objects = [collision_obj]

                # add planning scene into planning scene msg
                self.scene_result.world = plan_scene_world

                # publish to planning scene topic
                self.scene_pub.publish(self.scene_result)

        if self.robot_state == State.NOTSET:
            try:
                self.tf_base_arm = self.tf_buffer.lookup_transform('panda_link0',
                                                                   'panda_hand', rclpy.time.Time())
                self.curr_pos = [self.tf_base_arm.transform.translation.x,
                                 self.tf_base_arm.transform.translation.y,
                                 self.tf_base_arm.transform.translation.z]

            except TransformException as ex:
                self.get_logger().info(f'Could not transform : {ex}')

        # The following is IK for FINAL joint state.
        if self.robot_state == State.GO:
            # filling in IK request msg
            self.ik_states.group_name = "panda_arm"

            # PoseStamped msg
            self.ik_robot_states.pose = self.ik_pose

            # filling in robot state field in IK request
            self.ik_states.pose_stamped = self.ik_robot_states

            ik_future = self.ik_client.call_async(GetPositionIK.Request(ik_request=self.ik_states))
            await ik_future

            if ik_future.done() and self.robot_state != State.OBTAINED:
                # self.get_logger().info(f'{ik_future.result()}')
                if ik_future.result().error_code.val < -1:
                    self.get_logger().info(f'The planned trajectory is unexecutable')
                    self.robot_state = State.NOTSET
                else:
                    self.final_js = ik_future.result().solution.joint_state.position
                    self.robot_state = State.OBTAINED
                    self.send_goal()

    def send_goal(self):
        """
        Send the goal position to the robot.

        Args: None

        Returns: None
        """
        move_group_goal = MoveGroup.Goal()
        goal_msg = MotionPlanRequest()

        # change to PlanOnly
        plan_option = PlanningOptions()
        plan_option.plan_only = True
        move_group_goal.planning_options = plan_option

        # add time stamp
        time = self.get_clock().now().to_msg()
        work_msg = WorkspaceParameters()
        header_msg = Header()
        header_msg.stamp = time
        header_msg.frame_id = 'panda_link0'
        work_msg.header = header_msg

        # add workspace space limits
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

        # input desired starting position
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
        goal_msg.max_velocity_scaling_factor = 0.1
        goal_msg.max_acceleration_scaling_factor = 0.1
        move_group_goal.request = goal_msg

        # send the goal request
        self.send_pos.wait_for_server()

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

        # execute the trajectory after getting the result, if self.execute_immediately is true
        if self.execute_immediately is True:
            self.get_logger().info("Start executing trajectory")
            self.exe_trajectory()
            return
        else:
            self.robot_state = State.NOTSET

        # Only execute if robot user_start_config is not set
        if not self.user_start_config:
            # execute the trajectory after getting the result
            self.planned_trajectory = result.planned_trajectory
            self.get_logger().info("Start executing trajectory")
            self.exe_trajectory()
            return

        # Store the planned trajectory in text file.
        else:
            self.get_logger().info("Store the planned trajectory")
            self.traj_for_later = result.planned_trajectory

    def exe_trajectory(self):
        """
        Execute the trajectory.

        Args: None

        Returns: None
        """
        trajectory = ExecuteTrajectory.Goal()
        robot_traj = RobotTrajectory()
        joint_traj = JointTrajectory()

        # add time stamp and frame id
        time = self.get_clock().now().to_msg()
        header_msg = Header()
        header_msg.stamp = time
        header_msg.frame_id = 'panda_link0'
        joint_traj.header = header_msg

        # get joint trajectory from planned trajectory
        joint_pt_list = self.planned_trajectory.joint_trajectory.points

        # get joint names
        joint_names = self.planned_trajectory.joint_trajectory.joint_names

        # fill in the execute msg
        joint_traj.points = joint_pt_list
        joint_traj.joint_names = joint_names
        robot_traj.joint_trajectory = joint_traj
        trajectory.trajectory = robot_traj

        # send the request
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
        self.robot_state = State.NOTSET     # sub to joint_states, get the curr pos, and update it

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
        self.curr_joint_pos = msg.position
        return

    def balloon_callback(self, msg):
        """
        Subscription to the balloon_marker in RVIZ.

        Args: msg (visualization_msgs/msg/Marker): The marker published in RVIZ

        Returns: None
        """
        balloon_marker = msg
        self.balloon_position = balloon_marker.pose.position
        return


def main(args=None):
    rclpy.init(args=None)
    node = Mover()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
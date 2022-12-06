"""
Controls the visuals of the balloon

Publishers
----------
  + publishes to: "balloon_marker", type: Marker - publishes the balloon visual.

Subscribers
-----------
  + None

Services
--------
  + name: "balloon_drop", type: std_srvs/srv/Empty - drops the balloon from the placed position.

Parameters
----------
  + name: gravity, type: float - acceleration due to gravity, as defined in config/balloon.yaml
  + name: balloon_radius, type: float - radius of wheel, as defined in config/balloon.yaml

"""

from enum import Enum, auto
import math
import rclpy
from rclpy.node import Node
import rclpy.time
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from geometry_msgs.msg import Point, TransformStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from motion_planning_interfaces.srv import Place
from rclpy.action import ActionServer
from moveit_msgs.action import MoveGroup, ExecuteTrajectory


class State(Enum):
    """Current state of the system. Determines what timer do in each state."""

    START = auto()
    PLACE_BALLOON = auto()
    DROP_BALLOON = auto()


class Arena(Node):
    """Publish visuals of the balloon and arena."""

    def __init__(self):
        super().__init__('arena')
        self.count = 0
        self.frequency = 250.0
        self.timer = self.create_timer(1 / self.frequency, self.timer_callback)
        self.balloon_pub = self.create_publisher(Marker, "balloon_marker", 10)
        self.balloon_drop = self.create_service(
            Empty, "balloon_drop", self.drop_callback)
        self.balloon_drop = self.create_service(
            Place, "balloon_place", self.place_callback)

        self.state = State.START
        self.broadcaster = TransformBroadcaster(self)
        self.time = 0.0
        # HARD-CODED position of the balloon
        self.balloon_place_initial = Point()
        self.balloon_z_initial = self.balloon_place_initial.z
        # self.balloon_pos = self.balloon_place_initial

        self.declare_parameter("gravity", 3.5, ParameterDescriptor(
            description="Accel due to gravity, 9.8 by default."))
        self.declare_parameter("balloon_radius_x", 0.15/2, ParameterDescriptor(
            description="x radius of balloon"))
        self.declare_parameter("balloon_radius_y", 0.25/2, ParameterDescriptor(
            description="y radius of balloon"))
        self.accel_g = self.get_parameter(
            "gravity").get_parameter_value().double_value
        if self.accel_g < 0:
            print("Acceleration due to gravity must be positive! Correcting...")
            self.accel_g = abs(self.accel_g)
        if self.accel_g == 0:
            print("Acceleration due to gravity can't be 0! Defaulting...")
            self.accel_g = 3.5
        self.balloon_radius_x = self.get_parameter(
            "balloon_radius_x").get_parameter_value().double_value
        if self.balloon_radius_x < 0:
            print("Balloon radius must be positive! Correcting...")
            self.balloon_radius_x = abs(self.balloon_radius_x)
        if self.balloon_radius_x == 0:
            print("Balloon radius can't be 0! Defaulting...")
            self.balloon_radius_x = 0.15/2
        self.balloon_radius_y = self.get_parameter(
            "balloon_radius_y").get_parameter_value().double_value
        if self.balloon_radius_y < 0:
            print("Balloon radius must be positive! Correcting...")
            self.balloon_radius_y = abs(self.balloon_radius_y)
        if self.balloon_radius_y == 0:
            print("Balloon radius can't be 0! Defaulting...")
            self.balloon_radius_y = 0.25/2

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.world_balloon = TransformStamped()

        self.marker_balloon = Marker(type=2)
        self.marker_balloon.id = 5
        self.marker_balloon.header.stamp = self.get_clock().now().to_msg()
        self.marker_balloon.header.frame_id = "panda_link0"
        self.marker_balloon.color.r = 234.0 / 255.0
        self.marker_balloon.color.g = 170.0 / 255.0
        self.marker_balloon.color.b = 0.0 / 255.0
        self.marker_balloon.color.a = 1.0
        self.marker_balloon.action = 0
        self.marker_balloon.scale.x = 2.0 * self.balloon_radius_x
        self.marker_balloon.scale.y = 2.0 * self.balloon_radius_y
        self.marker_balloon.scale.z = self.marker_balloon.scale.x
        # self.marker_balloon.pose.position = self.balloon_place_initial

        self.time = 0.0

    def timer_callback(self):
        """Check different states to determine which function to call."""
        # print(self.accel_g)
        if self.state != State.START:
            self.world_balloon.header.stamp = self.get_clock().now().to_msg()
            self.world_balloon.header.frame_id = "panda_link0"
            self.world_balloon.child_frame_id = "balloon"
            self.world_balloon.transform.translation.x = self.marker_balloon.pose.position.x
            self.world_balloon.transform.translation.y = self.marker_balloon.pose.position.y
            self.world_balloon.transform.translation.z = self.marker_balloon.pose.position.z
            self.broadcaster.sendTransform(self.world_balloon)
            self.balloon_pub.publish(self.marker_balloon)

            if self.state == State.DROP_BALLOON:
                self.time = self.time + 1 / self.frequency
                self.marker_balloon.pose.position.z = self.balloon_z_initial - (
                    0.5 * self.accel_g * self.time**2)
                if self.marker_balloon.pose.position.z <= 0.0:
                    self.state = State.PLACE_BALLOON

            self.count += 1

    def drop_callback(self, request, response):
        """
        Drop balloon.

        Keyword Arguments:
        -----------------
            request -- type std_srvs.srv.Empty
            response -- type std_srvs.srv.Empty

        """
        if self.state == State.PLACE_BALLOON:
            self.state = State.DROP_BALLOON
        else:
            print("Place balloon first!")
        return response
    
    def place_callback(self, request, response):
        """
        Place balloon.

        Keyword Arguments:
        -----------------
            request -- type geometry.msgs.Point
            response -- type std_srvs.srv.Empty

        """
        self.balloon_place_initial = Point(x=request.balloon_x, y=request.balloon_y,
                                            z=request.balloon_z)
        self.marker_balloon.header.stamp = self.get_clock().now().to_msg()
        self.marker_balloon.header.frame_id = "panda_link0"
        self.marker_balloon.pose.position = self.balloon_place_initial
        self.balloon_z_initial = self.balloon_place_initial.z
        if self.state == State.START:
            self.state = State.PLACE_BALLOON
        return response

class Intercept(Node):
    def __init__(self):
        super().__init__("intercept_node")
        self.action_server = ActionServer(self, MoveGroup, "move_action", self.move_callback)
        self.execute_traj = ActionServer(self, ExecuteTrajectory, 'execute_trajectory', self.exe_callback)
        # self.action_server = ActionServer(self, MoveGroup, "move_action", self.move_callback)
    
    def move_callback(self, goal):
        print('move')
        self.get_logger().info(f"{goal.request}")
        result = MoveGroup.Result()
        return result
    
    def exe_callback(self, goal): 
        print('exe')
        self.get_logger().info(f"{goal.request}")
        result = ExecuteTrajectory.Result()
        return result

def intercept_entry(args=None):
    rclpy.init(args=None)
    # node = Intercept()
    node = Arena()
    rclpy.spin(node)
    rclpy.shutdown()

def main(args=None):
    """Create an arena node and spin."""
    rclpy.init(args=args)
    node = Intercept()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

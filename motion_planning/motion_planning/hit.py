"""
Predict the location of the balloon.

Also command the end-effector to go to the balloon position and tap it up.

PUBLISHER:
    + /cartesian_waypoint (geometry_msgs/msg/PoseArray) - Publish cartesian waypoints

SUBSCRIBERS:
    + /balloon_coords (geometry_msgs/msg/Point) - Subscribe to the balloon coordinates
    + /curr_ee_pos (geometry_msgs/msg/Point) - Subscribe to the end effector coordinates
    + /cart_cycle_complete (std_msgs/msg/Empty) - Subscribe to the state of simple_move node
"""

import rclpy
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseArray
from enum import Enum, auto
from rclpy.node import Node
from std_msgs.msg import Empty


class State(Enum):
    """Current state of the robot."""

    NOTSET = auto()
    GO = auto()
    STOP = auto()
    NOTPUB = auto()
    PUB = auto()


class hit(Node):
    """This node predicts the balloon trajectory. It will publish a predicted hit pose."""

    def __init__(self):
        """Initialize variables for hit node."""
        super().__init__('hit')

        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.balloonpos = self.create_subscription(Point,
                                                   'balloon_coords', self.balloon_callback, 10)
        self.curr_pos_sub = self.create_subscription(Point,
                                                     'curr_ee_pos', self.curr_pos_callback, 10)
        self.ee_pos_pub = self.create_publisher(PoseArray,
                                                'cartesian_waypoint', 10)
        self.state_sub = self.create_subscription(Empty,
                                                  'cart_cycle_complete', self.state_callback, 10)

        self.balloon_pos_x = []
        self.balloon_pos_y = []
        self.balloon_pos_z = []
        self.way_pts = PoseArray()
        self.state = State.NOTSET
        self.receive_state = State.NOTPUB
        self.balloon_pos = Point()
        self.move_to = Pose()
        self.last_time = 0.
        self.curr_time = 0.
        self.curr_pos = Point()
        self.offset = 0.1
        self.first_point = True
        self.state_cb_called = False

        # Transform from cam to robot base frame
        self.Trc = np.array([[1, 0, 0, 1.11],
                             [0, 0, 1, -1.7],
                             [0, -1, 0, 0.735],
                             [0, 0, 0, 1]])

        # Z threshold for letting it know when to record
        self.z_thresh = 1.225
        self.is_falling = False
        self.is_rising = False
        self.cycle_complete = State.NOTSET

    def state_callback(self, msg):
        """
        Determine the state of simple_move, by subscribing to /cart_cycle_complete.

        Args: msg (std_msgs/msg/Empty): Tells hit that simple_move just finished hitting the
                                        balloon once.

        Returns: None
        """
        print("state callback reached!")
        self.cycle_complete = State.NOTSET
        self.state = State.STOP
        self.first_point = True
        self.is_falling = False
        self.balloon_pos_x = []
        self.balloon_pos_y = []
        self.balloon_pos_z = []
        self.state_cb_called = True
        self.way_pts = PoseArray()

    def balloon_callback(self, msg):
        """
        Subscribe to /balloon_coords to get the coordinates of the ballon centroid.

        Args: msg (geometry_msgs/msg/Point): Centroid of balloon

        Returns: None
        """
        if msg.x != 0 and msg.y != 0 and msg.z != 0:
            self.balloon_pos.x = msg.x
            self.balloon_pos.y = msg.y
            self.balloon_pos.z = msg.z
            self.receive_state = State.PUB

        else:
            self.receive_state = State.NOTPUB

    def timer_callback(self):
        """
        Determine the state of the robot and balloons.

        Args: None

        Returns: None
        """
        # curve fit: To avoid difficulty with 3D curve parametrization
        #            and to predict the approximate location of the balloon
        #            try to predict x y position by fitting a line in
        #            x-y plane, then fit a parabola in y-z plane to figure
        #            out the z coordinate.

        # Replace the following coordinates with actual tracking data
        # Balloon pos in cam frame
        v_cam = np.array([self.balloon_pos.x, self.balloon_pos.y, self.balloon_pos.z, 1])

        # Balloon pos in robot base frame
        v_robot = self.Trc @ v_cam.reshape((4, 1))

        # Check if the z is within the threshold range
        self.check_rising(v_robot[2])
        self.check_falling(latest_z=v_robot[2])
        gathered_pts = 2  # Number of points to recieve from camera
        if self.receive_state == State.PUB \
                and len(self.balloon_pos_x) \
                < gathered_pts and self.is_falling:
            self.get_logger().info("Gathering Points")
            v_robot = v_robot[0:3]
            b_pos = v_robot

            if self.first_point:
                self.balloon_pos_x.append(b_pos[0][0])
                self.balloon_pos_y.append(b_pos[1][0])
                self.balloon_pos_z.append(b_pos[2][0])
                self.first_point = False

            if self.balloon_pos_x[0] != b_pos[0][0] and not self.first_point:
                self.balloon_pos_x.append(b_pos[0][0])
                self.balloon_pos_y.append(b_pos[1][0])
                self.balloon_pos_z.append(b_pos[2][0])

        elif len(self.balloon_pos_x) >= gathered_pts and self.receive_state == State.PUB:
            # Determining euler step prediction for velocity
            self.state = State.GO
            # Calculate the initial x and y velocity
            velx = (self.balloon_pos_x[1] - self.balloon_pos_x[0])/0.01
            vely = (self.balloon_pos_y[1] - self.balloon_pos_y[0])/0.01

        if self.state == State.GO and self.cycle_complete == State.NOTSET:
            self.get_logger().info("Starting Prediction")
            predicted_x = []
            predicted_y = []
            init_x = self.balloon_pos_x[0]
            init_y = self.balloon_pos_y[0]
            dt = 0.01
            for _ in range(12):
                init_x += velx * dt
                init_y += vely * dt
                predicted_x.append(init_x)
                predicted_y.append(init_y)

            # Choose the closest point
            pred_x = np.array(predicted_x)
            pred_y = np.array(predicted_y)
            distances = np.sqrt(np.power(pred_x - self.curr_pos.x, 2)
                                + np.power(pred_y - self.curr_pos.y, 2))
            indx = np.argmin(distances)

            # + self.offset
            self.move_to.position.x = float(pred_x[indx])

            # - self.offset
            self.move_to.position.y = float(pred_y[indx])
            self.move_to.position.z = float(0.25)
            self.move_to.orientation.w = 0.0
            self.move_to.orientation.x = 1.0
            self.move_to.orientation.y = 0.0
            self.move_to.orientation.z = 0.0
            self.way_pts.poses.append(self.move_to)

            print("Move To", self.move_to.position.x, self.move_to.position.y,
                  self.move_to.position.z)

            # Publish this to Inverse Kinematics and move the arm
            if self.cycle_complete == State.NOTSET:

                # + self.offset
                self.move_to.position.x = float(pred_x[indx])

                # - self.offset
                self.move_to.position.y = float(pred_y[indx])
                self.move_to.position.z = float(0.5)

                self.move_to.orientation.w = 0.0
                self.move_to.orientation.x = 1.0
                self.move_to.orientation.y = 0.0
                self.move_to.orientation.z = 0.0

                self.way_pts.poses.append(self.move_to)
                self.ee_pos_pub.publish(self.way_pts)
                self.cycle_complete = State.GO
                self.is_falling = False
                self.state_cb_called = False

    def check_falling(self, latest_z):
        """
        Determine if balloon is falling.

        Args: latest_z: The current height of the balloon

        Returns: None
        """
        if latest_z == self.Trc[2, -1]:
            self.is_falling = False
        else:
            if self.state_cb_called is False:
                self.is_falling = True if latest_z < self.z_thresh else False
            else:
                self.is_falling = False

    def check_rising(self, latest_z):
        """
        Determine if balloon is rising.

        Args: latest_z: The current height of the balloon

        Returns: None
        """
        if self.state_cb_called is True:
            self.state_cb_called = False if latest_z > self.z_thresh else True

    def curr_pos_callback(self, msg):
        """
        Subscribe to /curr_ee_pos to get the coordinates of the end-effector.

        Args: msg (geometry_msgs/msg/Point): Coordinates of end-effector

        Returns: None
        """
        self.curr_pos.x = msg.x
        self.curr_pos.y = msg.y
        self.curr_pos.z = msg.z
        return


def main(args=None):
    rclpy.init(args=None)
    node = hit()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

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
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseArray
from enum import Enum, auto
from rclpy.node import Node
import cv2 as cv
from std_msgs.msg import Empty


class KF:
    def __init__(self):
        '''Initialize kalman filter variable'''
        self.kf = cv.KalmanFilter(2, 2, 0)
        self.kf.measurementMatrix = np.array([[1, 0],
                                              [0, 1]], np.float32)
        self.kf.transitionMatrix = np.eye(2, dtype=np.float32)

    def kf_predict(self, coordX, coordY):
        ''' This function estimates the position of the object'''
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        self.kf.correct(measured)
        predicted = self.kf.predict()
        x, y = predicted[0], predicted[1]
        return x, y


class State(Enum):
    """
    Current state of the robot.
    """

    NOTSET = auto()
    GO = auto()
    STOP = auto()
    NOTPUB = auto()
    PUB = auto()


class hit(Node):

    def __init__(self):
        '''Initialize variables for hit node.'''
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
        self.kf = KF()
        self.curr_pos = Point()
        self.offset = 0.1
        self.first_point = True
        self.state_cb_called = False

        # transform from cam to robot base frame
        self.Trc = np.array([[1, 0, 0, 1.11],
                             [0, 0, 1, -1.7],
                             [0, -1, 0, 0.735],
                             [0, 0, 0, 1]])

        # Z Threshold for Letting it Know when to record
        self.z_thresh = 1.225
        self.is_falling = False
        self.is_rising = False
        self.cycle_complete = State.NOTSET

    def state_callback(self, msg):
        """
        Determine the state of simple_move, by subscribing to /cart_cycle_complete.

        Args: msg (Empty): Tells hit that simple_move just finished hitting the balloon once.

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
        Subscribes to /balloon_coords to get the coordinates of the ballon centroid.

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
        """

        # curve fit: To avoid difficulty with 3D curve parametrization
        #            and to predict the approximate location of the balloon
        #            try to predict x y position by fitting a line in
        #            x-y plane, then fit a parabola in y-z plane to figure
        #            out the z coordinate.

        # replace the following coordinates with actual tracking data
        # balloon pos in cam frame
        v_cam = np.array([self.balloon_pos.x, self.balloon_pos.y, self.balloon_pos.z, 1])

        # balloon pos in robot base frame
        v_robot = self.Trc @ v_cam.reshape((4, 1))

        # Check if the z is within the threshold range
        self.check_rising(v_robot[2])
        self.check_falling(latest_z=v_robot[2])
        gathered_pts = 2 # number of points to recieve from camera
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
            # determining euler step prediction for velocity
            self.state = State.GO
            # calculate the initial x and y velocity
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

            # choose the closest point
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

            # publish this to Inverse Kinematics and move the arm
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
        """Helper function to dtermine if balloon is falling."""
        if latest_z == self.Trc[2, -1]:
            self.is_falling = False
        else:
            if self.state_cb_called is False:
                self.is_falling = True if latest_z < self.z_thresh else False
            else:
                self.is_falling = False

    def check_rising(self, latest_z):
        """Helper function to determine if balloon is rising."""
        if self.state_cb_called is True:
            self.state_cb_called = False if latest_z > self.z_thresh else True

    def curr_pos_callback(self, msg):
        """
        Subscribes to /curr_ee_pos to get the coordinates of the end-effector.

        Args: msg (geometry_msgs/msg/Point): Coordinates of ee

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

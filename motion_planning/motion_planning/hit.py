import rclpy
import numpy as np
from scipy.optimize import curve_fit
import modern_robotics as mr
import simple_move as move
from enum import Enum, auto
from rclpy.node import Node
from motion_planning_interfaces.srv import GetPose


class State(Enum):
    """
    Current state of the robot.
    """

    NOTSET = auto()
    PREPARE = auto()
    IN_HIT = auto()


class hit(Node):

    def __init__(self):
        super().__init__('hit')

        timer_period = 0.001
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        Determine the state of the robot and balloons.
        """

        # Move the arm to preparation position
        # TODO: fit a trajectory to predict where the balloon
        #       will go
        #----------------------------------------------------
        # curve fit: To avoid difficulty with 3D curve parametrization
        #            and to predict the approximate location of the balloon
        #            try to predict x y position by fitting a line in
        #            x-y plane, then fit a parabola in y-z plane to figure
        #            out the z coordinate.

        # TODO: replace the following coordinates with actual tracking data
        x = np.random.rand(10)
        y = np.random.rand(10)
        z = np.random.rand(10)

        # fit a line in x-y plane
        def xy_line(x, k, b):
            return k*x + b

        xy_param, _ = curve_fit(xy_line, x, y)

        # fit a parabola in y-z plane
        def yz_parabola(y, a, b, c):
            return a*y**2 + b*y + c

        parabola_param, _ = curve_fit(yz_parabola, y, z)

        # When the balloon is within certain distance
        # position the robot for finer hit pose
        # TODO: fit the trajectory again since the balloon's
        #       trajectory may be erratic

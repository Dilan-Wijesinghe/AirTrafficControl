import rclpy
import math
import numpy as np
from scipy.optimize import curve_fit
from geometry_msgs.msg import Point, Pose
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
        self.balloon_pos = self.create_subscription(Point, 'balloon_coords', self.balloon_callback, 10)
        self.ee_pos_pub = self.create_publisher(Pose, 'set_pose', 10)

        self.balloon_pos_x = []
        self.balloon_pos_y = []
        self.balloon_pos_z = []
        self.balloon_pos = Point()
        self.move_to = Pose()

    def balloon_callback(self, msg):
        self.balloon_pos.x = msg.point.x
        self.balloon_pos.y = msg.point.y
        self.balloon_pos.z = msg.point.z


    def timer_callback(self):
        """
        Determine the state of the robot and balloons.
        """

        #----------------------------------------------------
        # curve fit: To avoid difficulty with 3D curve parametrization
        #            and to predict the approximate location of the balloon
        #            try to predict x y position by fitting a line in
        #            x-y plane, then fit a parabola in y-z plane to figure
        #            out the z coordinate.

        # replace the following coordinates with actual tracking data
        if len(self.balloon_pos_x) < 40:
            # transform from cam to robot base frame
            Rz = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])
            Tcr = np.array([[0, -1, 0, 1.64], [0, 0, 1, -0.61], [1, 0, 0, 0.18], [0, 0, 0, 1]])
            Tcr = np.matmul(Rz, Tcr)
            v_cam = np.array([self.balloon_pos.x, self.balloon_pos.y, self.balloon_pos.z]) # balloon pos in cam frame
            v_robot = np.matmul(Tcr, v_cam.reshape((3,1))) # balloon pos in robot base frame
            
            b_pos = v_robot
            self.balloon_pos_z.append(b_pos[2])
            self.balloon_pos_x.append(b_pos[0])
            self.balloon_pos_y.append(b_pos[1])

        x = self.balloon_pos_x
        y = self.balloon_pos_y
        z = self.balloon_pos_z
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
        # give a default z coordinate
        pred_z = 0.2
        parabola_a = parabola_param[0]
        parabola_b = parabola_param[1]
        parabola_c = parabola_param[2] - pred_z

        del_eqn = math.sqrt(parabola_b**2 - 4*parabola_a*parabola_c)
        y1 = (-parabola_b + del_eqn)/(2*parabola_a)
        y2 = (-parabola_b - del_eqn)/(2*parabola_a)

        line_k = xy_param[0]
        line_b = xy_param[1]
        x1 = (y1-line_b)/line_k
        x2 = (y2-line_b)/line_k

        # TODO: choose the x y that's further from the 
        # initial position of the balloon
        init_x = self.balloon_pos_x[0]
        init_y = self.balloon_pos_y[0]
        dist_1 = math.sqrt((x1-init_x)**2 + (y1-init_y)**2)
        dist_2 = math.sqrt((x2-init_x)**2 + (y2-init_y)**2)
        if dist_1 < dist_2:
            pred_x = x1
            pred_y = y1
        else:
            pred_x = x2
            pred_y = y2

        # move the ee to this position
        self.move_to.position.x = pred_x
        self.move_to.position.y = pred_y
        self.move_to.position.z = pred_z

        # face the end effector upward
        rot_ang = 0.0
        x_ang = np.pi/2
        y_ang = np.pi/2
        z_ang = 0.0

        qw = np.cos(rot_ang/2)
        qx = np.sin(rot_ang/2)*np.cos(x_ang)
        qy = np.sin(rot_ang/2)*np.cos(y_ang)
        qz = np.sin(rot_ang/2)*np.cos(z_ang)

        self.move_to.orientation.w = qw
        self.move_to.orientation.x = qx
        self.move_to.orientation.y = qy
        self.move_to.orientation.z = qz

        # publish this to Inverse Kinematics and move the arm
        self.ee_pos_pub.publish(self.move_to)

        # TODO: Generalize the ee pose

        

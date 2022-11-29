import rclpy
import math
import numpy as np
from scipy.optimize import curve_fit
from geometry_msgs.msg import Point, Pose
# import simple_move as move
from enum import Enum, auto
from rclpy.node import Node
import matplotlib.pyplot as plt  
# from motion_planning_interfaces.srv import GetPose


class State(Enum):
    """
    Current state of the robot.
    """

    NOTSET = auto()
    GO = auto()
    STOP = auto()


class hit(Node):

    def __init__(self):
        super().__init__('hit')

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.balloon_pos = self.create_subscription(Point, 'balloon_coords', self.balloon_callback, 10)
        self.ee_pos_pub = self.create_publisher(Pose, 'set_pose', 10)

        self.balloon_pos_x = []
        self.balloon_pos_y = []
        self.balloon_pos_z = []
        self.state = State.NOTSET
        self.balloon_pos = Point()
        self.move_to = Pose()

    def balloon_callback(self, msg):
        self.balloon_pos.x = msg.x
        self.balloon_pos.y = msg.y
        self.balloon_pos.z = msg.z


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
        if len(self.balloon_pos_x) < 200:
            # transform from cam to robot base frame
            Tcr = np.array([[1,0,0,-(0.92+0.56)], [0,-1,0,-0.03], [0,0,-1,-(1.85-0.09)], [0,0,0,1]])
            v_cam = np.array([self.balloon_pos.x, self.balloon_pos.y, self.balloon_pos.z, 0]) # balloon pos in cam frame
            v_robot = np.matmul(Tcr, v_cam.reshape((4,1))) # balloon pos in robot base frame
            
            # The points used for curve fit are already in the robot base frame
            v_robot = v_robot[0:3]
            
            b_pos = v_robot
            self.balloon_pos_z.append(b_pos[2][0])
            self.balloon_pos_x.append(b_pos[0][0])
            self.balloon_pos_y.append(b_pos[1][0])
        
        elif self.state == State.NOTSET:
            self.state = State.GO
        
        if self.state == State.GO:
            x = np.array(self.balloon_pos_x)
            y = np.array(self.balloon_pos_y)
            z = np.array(self.balloon_pos_z)

            # # calculate COM for all the points
            # comx = np.sum(np.array(self.balloon_pos_x))/len(self.balloon_pos_x)
            # comy = np.sum(np.array(self.balloon_pos_y))/len(self.balloon_pos_x)
            # comz = np.sum(np.array(self.balloon_pos_z))/len(self.balloon_pos_x)

            # # calculate the distance between points and COM
            # dist_x = np.array(self.balloon_pos_x) - comx
            # dist_y = np.array(self.balloon_pos_y) - comy
            # dist_z = np.array(self.balloon_pos_z) - comz
            # total_dist = np.sqrt(np.power(dist_x, 2) + np.power(dist_y, 2) + np.power(dist_z, 2))

            # x = x[total_dist < 0.2]
            # y = y[total_dist < 0.2]
            # z = z[total_dist < 0.2]

            # fit a line in x-y plane
            def xz_line(x, k, b):
                return k*x + b

            xz_param, _ = curve_fit(xz_line, x, y)

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
            x1 = (y1 - line_b)/line_k
            x2 = (y2 - line_b)/line_k

            # TODO: choose the x y that's further from the 
            # initial position of the balloon
            init_x = 0
            init_y = 0
            dist_1 = math.sqrt((x1-init_x)**2 + (y1-init_y)**2)
            dist_2 = math.sqrt((x2-init_x)**2 + (y2-init_y)**2)
            if dist_1 < dist_2:
                pred_x = x
                pred_y = y1
            else:
                pred_x = x
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
            self.state = State.STOP

            fig = plt.figure()
            ax = plt.axes(projection='3d')
            ax.plot(x, y, z, 'xk')
            ax.plot(x[0], y[0], z[0], 'ob')
            ax.plot(pred_x, pred_y, pred_z, 'xr')
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_zlabel('z')
            plt.show()

            self.get_logger().info("predicted point" + str(self.move_to))

        # TODO: Generalize the ee pose

def main(args=None):
    rclpy.init(args=None)
    node = hit()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
 

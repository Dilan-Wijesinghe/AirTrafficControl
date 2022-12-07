import rclpy
from .ekf_predict import ekf
import numpy as np
from geometry_msgs.msg import Point, Pose
from enum import Enum, auto
from rclpy.node import Node
import matplotlib.pyplot as plt  
import cv2 as cv


class KF:
    def __init__(self):
        self.kf = cv.KalmanFilter(6,3,0)
        # self.kf.measurementMatrix = np.array([[1,0,0],
        #                                       [0,1,0],
        #                                       [0,0,1]],np.float32)

        # self.kf.transitionMatrix = np.eye(3, dtype=np.float32)

        self.kf.measurementMatrix = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0]],np.float32)

        self.kf.transitionMatrix = np.array([[1,0,0,1,0,0],[0,1,0,0,1,0],[0,0,1,0,0,1],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]],np.float32)
    def kf_predict(self, coordX, coordY,coordZ):
        ''' This function estimates the position of the object'''
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)],[np.float32(coordZ)]])
        self.kf.correct(measured)
        predicted = self.kf.predict()
        x, y ,z = predicted[0], predicted[1], predicted[2]
        return x, y, z

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
        super().__init__('hit')

        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.balloonpos = self.create_subscription(Point, 'balloon_coords', self.balloon_callback, 10)
        self.ee_pos_pub = self.create_publisher(Pose, 'set_pose', 10)
        # self.ee_pos_pub = self.create_publisher(Pose, 'cartesian_waypoint', 10)

        self.balloon_pos_x = []
        self.balloon_pos_y = []
        self.balloon_pos_z = []
        self.state = State.NOTSET
        self.receive_state = State.NOTPUB
        self.balloon_pos = Point()
        self.move_to = Pose()
        self.last_time = 0.
        self.curr_time = 0.
        self.kf = KF()

    def balloon_callback(self, msg):
        if msg.x != 0 and msg.y !=0 and msg.z != 0:
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

        #----------------------------------------------------
        # curve fit: To avoid difficulty with 3D curve parametrization
        #            and to predict the approximate location of the balloon
        #            try to predict x y position by fitting a line in
        #            x-y plane, then fit a parabola in y-z plane to figure
        #            out the z coordinate.

        # replace the following coordinates with actual tracking data
        gathered_pts = 8
        if self.receive_state == State.PUB and len(self.balloon_pos_x) < gathered_pts:
            # transform from cam to robot base frame
            Trc = np.array([[1,0,0,1.11], 
                            [0,0,1,-1.7], 
                            [0,-1,0,0.735], 
                            [0,0,0,1]])
            v_cam = np.array([self.balloon_pos.x, self.balloon_pos.y, self.balloon_pos.z, 1]) # balloon pos in cam frame
            v_robot = Trc @ v_cam.reshape((4,1)) # balloon pos in robot base frame
             
            # The points used for curve fit are already in the robot base frame
            v_robot = v_robot[0:3]
            
            b_pos = v_robot

            print("balloon in robot frame: " + str(b_pos))
            print("balloon in cam frame: " + str(v_cam))
            
            self.balloon_pos_x.append(b_pos[0][0])
            self.balloon_pos_y.append(b_pos[1][0])
            self.balloon_pos_z.append(b_pos[2][0])
        
        elif len(self.balloon_pos_x) >= gathered_pts and self.receive_state == State.PUB:
            self.state = State.GO
        
        if self.state == State.GO:
            x = np.array(self.balloon_pos_x)
            y = np.array(self.balloon_pos_y)
            z = np.array(self.balloon_pos_z)

            print("Shape is:", len(self.balloon_pos_x))
            for i in range(len(self.balloon_pos_x)):
                pt_x = self.balloon_pos_x[i]
                pt_y = self.balloon_pos_y[i]
                pt_z = self.balloon_pos_z[i]
                predicted = self.kf.kf_predict(pt_x, pt_y, pt_z)
            predicted_x=[]
            predicted_y=[]
            predicted_z=[]
            for i in range(4):
                predicted = self.kf.kf_predict(predicted[0], predicted[1], predicted[2] )
                predicted_x.append(predicted[0][0])
                predicted_y.append(predicted[1][0])
                predicted_z.append(predicted[2][0])

            self.move_to.position.x = float(predicted[0][0])
            self.move_to.position.y = float(predicted[1][0])
            self.move_to.position.z = float(0.3)
            # self.move_to.position.z = float(predicted[2][0])

            pred = predicted
            print("predicted final point: " + str(pred))

            # face the end effector upward
            rot_ang = np.pi/2
            x_ang = 0.3
            y_ang = 0.0
            z_ang = 0.0

            qw = np.cos(rot_ang/2)
            qx = np.sin(rot_ang/2)*np.cos(x_ang)
            qy = np.sin(rot_ang/2)*np.cos(y_ang)
            qz = np.sin(rot_ang/2)*np.cos(z_ang)

            self.move_to.orientation.w = 0.0
            self.move_to.orientation.x = 1.0
            self.move_to.orientation.y = 0.0
            self.move_to.orientation.z = 0.0

            print("Move To" , self.move_to.position.x, self.move_to.position.y, \
                              self.move_to.position.z)

            # publish this to Inverse Kinematics and move the arm
            self.ee_pos_pub.publish(self.move_to)
            # self.state = State.STOP

            fig = plt.figure()
            ax = plt.axes(projection='3d')
            ax.plot(x, y, z, 'xk')
            ax.plot(x[0], y[0], z[0], 'ob')
            ax.plot(predicted_x, predicted_y,predicted_z, 'xr')
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_zlabel('z')
            plt.show()

            self.get_logger().info("predicted point" + str(self.move_to))

            ########### Extended Kalman Filter Prediction ###############
            # Calculate x y z velocity of balloon
            # using collected points. 
            diffx = np.diff(np.array(self.balloon_pos_x))
            velx = np.sum(diffx)/(diffx.shape[0] * self.timer_period)

            diffy = np.diff(np.array(self.balloon_pos_y))
            vely = np.sum(diffy)/(diffy.shape[0] * self.timer_period)

            diffz = np.diff(np.array(self.balloon_pos_z))
            velz = np.sum(diffz)/(diffz.shape[0] * self.timer_period)

            # prediction using EKF
            pred_pts_x = []
            pred_pts_y = []
            pred_pts_z = []
            predicted_num = 5
            state_estimate_k_minus_1 = np.array([x[-1], y[-1], z[-1]])
            control_k_minus_1 = np.array([velx, vely, velz])
            for i in range(predicted_num):
                predicted_pt = ekf(state_estimate_k_minus_1, control_k_minus_1, \
                                               velx, vely, velz)
                state_estimate_k_minus_1 = predicted_pt
                pred_pts_x.append(predicted_pt[0])
                pred_pts_y.append(predicted_pt[1])
                pred_pts_z.append(predicted_pt[2])

            # fig = plt.figure()
            # ax = plt.axes(projection='3d')
            # ax.plot(x, y, z, 'xk')
            # ax.plot(x[0], y[0], z[0], 'ob')
            # ax.plot(pred_pts_x, pred_pts_y,pred_pts_z, 'xr')
            # ax.set_xlabel('x')
            # ax.set_ylabel('y')
            # ax.set_zlabel('z')
            # ax.set_title('EKF prediction')
            # plt.show()

            # ros2 topic pub -1 /set_pose geometry_msgs/msg/Pose 
            # "{position: {x: 0.4, y: 0.0, z: 0.3}, orientation: {x: 1.0, y: 0., z: 0., w: 0.}}"

                


def main(args=None):
    rclpy.init(args=None)
    node = hit()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
 
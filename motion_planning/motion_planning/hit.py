# import rclpy
# import math
# import numpy as np
# from scipy.optimize import curve_fit
# from geometry_msgs.msg import Point, Pose
# # import simple_move as move
# from enum import Enum, auto
# from rclpy.node import Node
# import matplotlib.pyplot as plt  
# import cv2 as cv
# # from motion_planning_interfaces.srv import GetPose
# class KF:
#     def __init__(self):
#         self.kf = cv.KalmanFilter(3,3,0)
#         self.kf.measurementMatrix = np.array([[1,0,0],
#                                               [0,1,0],
#                                               [0,0,1]],np.float32)

#         self.kf.transitionMatrix = np.eye(3, dtype=np.float32)

#     def kf_predict(self, coordX, coordY,coordZ):
#         ''' This function estimates the position of the object'''
#         measured = np.array([[np.float32(coordX)], [np.float32(coordY)],[np.float32(coordZ)]])
#         self.kf.correct(measured)
#         predicted = self.kf.predict()
#         x, y ,z = predicted[0], predicted[1], predicted[2]
#         return x, y, z

# class State(Enum):
#     """
#     Current state of the robot.
#     """

#     NOTSET = auto()
#     GO = auto()
#     STOP = auto()
#     NOTPUB = auto()
#     PUB = auto()


# class hit(Node):

#     def __init__(self):
#         super().__init__('hit')

#         timer_period = 0.01
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.balloonpos = self.create_subscription(Point, 'balloon_coords', self.balloon_callback, 10)
#         self.ee_pos_pub = self.create_publisher(Pose, 'set_pose', 10)

#         self.balloon_pos_x = []
#         self.balloon_pos_y = []
#         self.balloon_pos_z = []
#         self.state = State.NOTSET
#         self.receive_state = State.NOTPUB
#         self.balloon_pos = Point()
#         self.move_to = Pose()

#         self.kf = KF()

#     def balloon_callback(self, msg):
#         if msg.x != 0 and msg.y !=0 and msg.z != 0:
#             self.balloon_pos.x = msg.x
#             self.balloon_pos.y = msg.y
#             self.balloon_pos.z = msg.z
            
#             self.receive_state = State.PUB

#         else:
#             self.receive_state = State.NOTPUB
        


#     def timer_callback(self):
#         """
#         Determine the state of the robot and balloons.
#         """

#         #----------------------------------------------------
#         # curve fit: To avoid difficulty with 3D curve parametrization
#         #            and to predict the approximate location of the balloon
#         #            try to predict x y position by fitting a line in
#         #            x-y plane, then fit a parabola in y-z plane to figure
#         #            out the z coordinate.

#         # replace the following coordinates with actual tracking data
#         gathered_pts = 30
#         predict_num = 40

#         if self.receive_state == State.PUB and len(self.balloon_pos_x) < gathered_pts:
#             # transform from cam to robot base frame
#             Trc = np.array([[1,0,0,1.11], 
#                             [0,0,1,-1.7], 
#                             [0,-1,0,0.735], 
#                             [0,0,0,1]])
#             v_cam = np.array([self.balloon_pos.x, self.balloon_pos.y, self.balloon_pos.z, 1]) # balloon pos in cam frame
#             v_robot = Trc @ v_cam.reshape((4,1)) # balloon pos in robot base frame
             
#             # The points used for curve fit are already in the robot base frame
#             v_robot = v_robot[0:3]
            
#             b_pos = v_robot

#             print("balloon in robot frame: " + str(b_pos))
#             print("balloon in cam frame: " + str(v_cam))
            
#             self.balloon_pos_x.append(b_pos[0][0])
#             self.balloon_pos_y.append(b_pos[1][0])
#             self.balloon_pos_z.append(b_pos[2][0])
        
#         elif len(self.balloon_pos_x) >= gathered_pts and self.receive_state == State.PUB:
#             self.state = State.GO
        
#         if self.state == State.GO:
#             x = np.array(self.balloon_pos_x)
#             y = np.array(self.balloon_pos_y)
#             z = np.array(self.balloon_pos_z)

#             print("Shape is:", len(self.balloon_pos_x))
#             coords = [[], [], []]
#             for i in range(len(self.balloon_pos_x)):
#                 pt_x = self.balloon_pos_x[i]
#                 pt_y = self.balloon_pos_y[i]
#                 pt_z = self.balloon_pos_z[i]
#                 coords[0].append(pt_x)
#                 coords[1].append(pt_y)
#                 coords[2].append(pt_z)
#                 # predicted = self.kf.kf_predict(pt_x, pt_y, pt_z)
#                 print(coords[0] ,coords[1], coords[2])
#             predicted = self.kf.kf_predict(coords[0], coords[1], coords[2])
            
#             # the for loop gives the trajectory prediction
#             predx_list = []
#             predy_list = []
#             predz_list = []
#             for i in range(predict_num):
#                 predicted = self.kf.kf_predict(predicted[0], predicted[1], predicted[2] )
#                 predx_list.append(float(predicted[0]))
#                 predy_list.append(float(predicted[1]))
#                 predz_list.append(float(predicted[2]))
            
#             # print(predx_list)

#             self.move_to.position.x = float(predicted[0][0])
#             self.move_to.position.y = float(predicted[1][0])
#             self.move_to.position.z = float(predicted[2][0])

#             #print("predicted final point: " + str(pred))

#             # face the end effector upward
#             rot_ang = np.pi/2
#             x_ang = 0
#             y_ang = 0
#             z_ang = 0.0

#             qw = np.cos(rot_ang/2)
#             qx = np.sin(rot_ang/2)*np.cos(x_ang)
#             qy = np.sin(rot_ang/2)*np.cos(y_ang)
#             qz = np.sin(rot_ang/2)*np.cos(z_ang)

#             self.move_to.orientation.w = qw
#             self.move_to.orientation.x = qx
#             self.move_to.orientation.y = qy
#             self.move_to.orientation.z = qz

#             print("Move To" , self.move_to.position.x, self.move_to.position.y, \
#                               self.move_to.position.z)
            
#             print(predicted)
#             # publish this to Inverse Kinematics and move the arm
#             # self.ee_pos_pub.publish(self.move_to)
#             # self.state = State.STOP

#             fig = plt.figure()
#             ax = plt.axes(projection='3d')
#             ax.plot(x, y, z, 'xk')
#             ax.plot(x[0], y[0], z[0], 'ob')
#             ax.plot(predx_list, predy_list, predz_list, 'xr')
#             ax.set_xlabel('x')
#             ax.set_ylabel('y')
#             ax.set_zlabel('z')
#             plt.show()

#             self.get_logger().info("predicted point" + str(self.move_to))

#         # TODO: Generalize the ee pose

# def main(args=None):
#     rclpy.init(args=None)
#     node = hit()
#     rclpy.spin(node)


# if __name__ == '__main__':
#     main()
 
import rclpy
import math
import numpy as np
from scipy.optimize import curve_fit
from geometry_msgs.msg import Point, Pose
# import simple_move as move
from enum import Enum, auto
from rclpy.node import Node
import matplotlib.pyplot as plt  
import cv2 as cv
# from motion_planning_interfaces.srv import GetPose
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

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.balloonpos = self.create_subscription(Point, 'balloon_coords', self.balloon_callback, 10)
        self.ee_pos_pub = self.create_publisher(Pose, 'cartesian_waypoint', 10)

        self.balloon_pos_x = []
        self.balloon_pos_y = []
        self.balloon_pos_z = []
        self.state = State.NOTSET
        self.receive_state = State.NOTPUB
        self.balloon_pos = Point()
        self.move_to = Pose()

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
        gathered_pts = 75
        if self.receive_state == State.PUB and len(self.balloon_pos_x) < gathered_pts:
            # transform from cam to robot base frame
            Trc = np.array([[1,0,0,1.11], 
                            [0,0,1,-1.7], 
                            [0,-1,0,0.735], 
                            [0,0,0,1]])
            # Rrc = Trc[0:3, 0:3]
            # prc = Trc[:, 3]
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
            for i in range(10):
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

            self.move_to.orientation.w = qw
            self.move_to.orientation.x = qx
            self.move_to.orientation.y = qy
            self.move_to.orientation.z = qz

            print("Move To" , self.move_to.position.x, self.move_to.position.y, \
                              self.move_to.position.z)

            # publish this to Inverse Kinematics and move the arm
            self.ee_pos_pub.publish(self.move_to)
            # self.state = State.STOP

            fig = plt.figure()
            ax = plt.axes(projection='3d')
            ax.plot(x, y, z, 'xk')
            ax.plot(x[0], y[0], z[0], 'ob')
            # ax.plot(pred[0], pred[1], pred[2], 'xr')
            ax.plot(predicted_x, predicted_y,predicted_z, 'xr')
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
 
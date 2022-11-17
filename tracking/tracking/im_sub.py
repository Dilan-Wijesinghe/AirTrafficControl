# Imports 
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

# Get the number of cameras available
def count_cameras():
    max_tested = 100
    for i in range(max_tested):
        temp_camera = cv.VideoCapture(i)
        if temp_camera.isOpened():
            temp_camera.release()
            continue
        return i

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        # Creating the Subscriber, which receives info from the video_frames topic
        # self.vid_sub = self.create_subscription(Image, 'video_frames', self.sub_callback, 10)
        self.vid_sub = self.create_subscription(Image, '/camera/color/image_raw', \
                                                self.sub_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_rect_raw', \
                                             self.depth_callback, 10)
        self.bridge = CvBridge()

        # Publisher that publishes to the video_frames topic
        self.vid_pub = self.create_publisher(Image, 'video_frames', 10)
        
        # self.T = 100 # Timer Frequency 
        # self.timer = self.create_timer(1/self.T, self.timer_callback) # Create Timer
        
        # Create a VideoCapture object 
        # The argument '0' gives the default webcam
        # self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge() # Make a bridge to go between ROS and OpenCV Images

        # Count the number of cameras
        self.get_logger().info(f"Number of Cameras is: {count_cameras()}")

        self.kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(3,3))
        self.bgsub = cv.createBackgroundSubtractorMOG2(detectShadows=False)

    # """
    # Callback function
    # TODO: 
    # """
    # def timer_callback(self):
    #     ret, frame = self.cap.read()
    #     if ret:
    #         # Publish the image using 'cv2_to_imgmsg'
    #         self.vid_pub.publish(self.bridge.cv2_to_imgmsg(frame))
        
    #     # Display message
    #     self.get_logger().info("Publishing Video Frame", once = True)

    def sub_callback(self, data):
        self.get_logger().info("Receiving video_frames", once=True)
        curr_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='mono8') # Convert ROS image msg to OpenCV image
        # frame_HSV = cv.cvtColor(curr_frame, cv.COLOR_BGR2HSV)
        # frame_thresh = cv.inRange(frame_HSV (low_h, low_s, low_v), (high_h, high_s, high_v)

        # Background Subtraction
        # im_rgb = cv2.cvtColor(curr_frame, cv2.COLOR_RGB2BGR)
        fgmask = self.bgsub.apply(curr_frame)
        fgmask = cv.morphologyEx(fgmask, cv.MORPH_OPEN, self.kernel)

        # Hough Circle Transform
        circles = cv.HoughCircles(curr_frame, cv.HOUGH_GRADIENT, 4, 200,
                                  param1=100, param2=200, minRadius=80,maxRadius=200)
        
        if circles is None:
            print("No circles!")
            pass
        else:
            print(circles)
            circles = np.uint16(np.around(circles))

            for i in circles[0,:]:
                cv.circle(curr_frame, (i[0], i[1]), i[2], (0,255,0), 2)
                cv.circle(curr_frame, (i[0], i[1]), 2, (0, 0, 255), 3)
        
        self.vid_pub.publish(self.bridge.cv2_to_imgmsg(curr_frame)) # Publish msg 
        # self.get_logger().info(self.bridge.cv_to_imgmsg(im_rgb), once=True)

        cv.imshow("Camera", curr_frame) # Display image
        # cv.imshow("FG Mask", fgmask) # Display foreground mask
        cv.waitKey(1)
    
    def depth_callback(self, data):
        self.get_logger().info("Receiving depth frames", once=True)
        curr_frame = self.bridge.imgmsg_to_cv2(data)
        depth_img = cv.applyColorMap(cv.convertScaleAbs(curr_frame, alpha=0.3), cv.COLORMAP_JET)
        self.vid_pub.publish(self.bridge.cv2_to_imgmsg(depth_img))

        # cv.imshow("Depth", depth_img) # Display depth image
        cv.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



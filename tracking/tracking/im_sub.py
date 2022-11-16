# Imports 
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Get the number of cameras available
def count_cameras():
    max_tested = 100
    for i in range(max_tested):
        temp_camera = cv2.VideoCapture(i)
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

        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
        self.fgbg = cv2.createBackgroundSubtractorMOG2(detectShadows=False)

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
        curr_frame = self.bridge.imgmsg_to_cv2(data) # Convert ROS image msg to OpenCV image
        im_rgb = cv2.cvtColor(curr_frame, cv2.COLOR_RGB2BGR)
        fgmask = self.fgbg.apply(im_rgb)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, self.kernel)
        cv2.imshow('frame', fgmask)
        self.vid_pub.publish(self.bridge.cv2_to_imgmsg(im_rgb)) # Publish msg 
        # self.get_logger().info(self.bridge.cv2_to_imgmsg(im_rgb), once=True)

        cv2.imshow("camera", im_rgb) # Display Image
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



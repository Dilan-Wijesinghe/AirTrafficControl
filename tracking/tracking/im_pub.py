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


"""
TODO:
"""
class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        # Publisher that publishes to the video_frames topic
        self.vid_pub = self.create_publisher(Image, 'video_frames', 10)
        self.T = 100 # Timer Frequency 
        self.timer = self.create_timer(1/self.T, self.timer_callback) # Create Timer
        # Create a VideoCapture object 
        # The argument '0' gives the default webcam
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge() # Make a bridge to go between ROS and OpenCV Images

        # Count the number of cameras
        self.get_logger().info(f"Number of Cameras is: {count_cameras()}")

    """
    Callback function
    TODO: 
    """
    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret:
            # Publish the image using 'cv2_to_imgmsg'
            self.vid_pub.publish(self.bridge.cv2_to_imgmsg(frame))
        
        # Display message
        self.get_logger().info("Publishing Video Frame", once = True)


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()




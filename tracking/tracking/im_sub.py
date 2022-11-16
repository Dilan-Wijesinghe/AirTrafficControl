# Imports 
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        # Creating the Subscriber, which receives info from the video_frames topic
        # self.vid_sub = self.create_subscription(Image, 'video_frames', self.sub_callback, 10)
        self.vid_sub = self.create_subscription(Image, '/camera/color/image_raw', \
                                                self.sub_callback, 10)
        self.bridge = CvBridge()

    def sub_callback(self, data):
        self.get_logger().info("Receiving video_frames", once=True)
        curr_frame = self.bridge.imgmsg_to_cv2(data) # Convert ROS image msg to OpenCV image
        im_rgb = cv2.cvtColor(curr_frame, cv2.COLOR_RGB2BGR)
        cv2.imshow("camera", im_rgb) # Display Image
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



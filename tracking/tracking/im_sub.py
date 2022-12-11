# Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
import cv2 as cv
import numpy as np
from enum import Enum, auto
import pyrealsense2 as rs


# Get the number of cameras available
def count_cameras():
    """
    Get the number of cameras available.

    Args: None

    Returns: None
    """
    max_tested = 100
    for i in range(max_tested):
        temp_camera = cv.VideoCapture(i)
        if temp_camera.isOpened():
            temp_camera.release()
            continue
        return i


class State(Enum):
    """Current state of the robot."""

    NOTPUB = auto()
    PUB = auto()


class ImageSubscriber(Node):
    """
    This node subscribes to certain topics relevant to the camera node.

    It also implements OpenCV modifications to the incoming camera feed.
    """

    def __init__(self):
        """Initialize variables needed in other functions, publishers, and subscribers."""
        super().__init__('image_subscriber')
        self.vid_sub = self.create_subscription(Image, 'camera/color/image_raw',
                                                self.sub_callback, 10)
        self.depth_sub = self.create_subscription(Image, 'camera/aligned_depth_to_color/image_raw',
                                                  self.depth_callback, 10)
        self.intr_sub = self.create_subscription(CameraInfo,
                                                 'camera/aligned_depth_to_color/camera_info',
                                                 self.intr_callback, 10)
        self.coords_pub = self.create_publisher(Point, 'balloon_coords', 10)

        # Publisher that publishes to the video_frames topic
        self.vid_pub = self.create_publisher(Image, 'video_frames', 10)

        self.bridge = CvBridge()  # Make a bridge to go between ROS and OpenCV Images

        # Count the number of cameras
        self.get_logger().info(f"Number of Cameras is: {count_cameras()}")

        self.kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (3, 3))
        # History -> How long to keep the frames (num iterations of loop)
        # varThreshold -> How sensitive to look at the foreground objects.
        #              -> Low varThreshold is more sensitive, high is less
        # detectShadows-> Boolean determining whether to detect shadows
        self.hist = 1000
        varThresh = 5
        self.bgsub = cv.createBackgroundSubtractorMOG2(history=self.hist, varThreshold=varThresh,
                                                       detectShadows=False)
        self.state = State.NOTPUB

        # Frame Instantiation
        self.intr = None
        self.color_frame = None
        self.depth_frame = None
        self.fgmask = None
        self.bg_removed = None
        self.z_pts = np.zeros((self.hist, 1))
        self.curr_pt = 0

    def sub_callback(self, data):
        """
        Perform several OpenCV operations on the frame of the colored raw image.

        Args: data (sensor_msgs/msg/Image)

        Returns: None
        """
        try:
            self.get_logger().info("Receiving video_frames", once=True)

            # Convert ROS image msg to OpenCV image
            curr_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            self.color_frame = cv.GaussianBlur(curr_frame, (11, 11), 0)
            self.vid_pub.publish(self.bridge.cv2_to_imgmsg(self.color_frame))

            self.remove_bg(clip_dist=5.0)  # Remove background

            # --- Color Variations ---
            frame_HSV = cv.cvtColor(self.bg_removed, cv.COLOR_BGR2HSV)
            frame_HSV_not = cv.bitwise_not(frame_HSV)

            frame_thresh = cv.inRange(frame_HSV_not, (35, 6, 10), (171, 131, 126))

            # Get Real Moving Object
            self.bg_sub(frame_thresh)  # Apply Background subtraction

            # Detect Contours
            contours, _ = cv.findContours(self.fgmask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            self.coords = Point()

            areas = [cv.contourArea(cont) for cont in contours]  # Loop over the contours
            MaxCont = contours[np.argmax(areas)]  # And find areas. Then find Max

            centroids = []

            # --- Drawing Onto the Frames ---
            ((x, y), r) = cv.minEnclosingCircle(MaxCont)
            if self.intr:
                if r > 10:  # Change depending on size and distance from camera
                    self.state = State.PUB  # Change ability to publish
                    # --- Current Moment for Centroid Finding ---
                    for cont in contours:
                        currMoment = cv.moments(cont)
                        cx = int(currMoment['m10']/currMoment['m00'])
                        cy = int(currMoment['m01']/currMoment['m00'])
                        centroids.append([cx, cy])
                        self.MaxCentroid = centroids[np.argmax(areas)]
                        cv.circle(self.color_frame, (int(x), int(y)), int(r), (0, 255, 255), 2)
                        cv.circle(self.color_frame, self.MaxCentroid, 10, (24, 146, 221), -1)
                else:
                    self.state = State.NOTPUB  # If it is too small, do not publish

            # fgmask 3 channeled
            fgmask_3 = cv.cvtColor(self.fgmask, cv.COLOR_GRAY2BGR)
            frame_thresh3 = cv.cvtColor(frame_thresh, cv.COLOR_GRAY2BGR)

            # Stack all three frames
            stacked = np.vstack((fgmask_3, frame_thresh3, self.color_frame))
            cv.imshow("Stacked", cv.resize(stacked, None, fx=0.65, fy=0.65))

            # --- Get Real Coords ---
            if len(areas) != 0:
                if self.intr:
                    depth = self.depth_frame[self.MaxCentroid[1], self.MaxCentroid[0]]
                    x_, y_, z_ = self.conv_to_real_coords(self.MaxCentroid[0],
                                                          self.MaxCentroid[1], depth)
                    self.coords.x = x_
                    self.coords.y = y_
                    self.coords.z = z_

                    if self.state == State.PUB:
                        self.z_pts[self.curr_pt][0] = z_
                        self.check_falling(z_)
                        self.curr_pt += 1
                        if self.curr_pt >= self.hist:  # Reset if at the point limit
                            self.curr_pt = 0
                        self.get_logger().info(f"Real Coords are: \
                            {self.coords.x, self.coords.y, self.coords.z}")
                        self.coords_pub.publish(self.coords)
            else:
                print("Cannot Calculate Centroid Depth")

            cv.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(e)
            return

        cv.waitKey(1)

    def check_falling(self, z):
        """
        Check if the balloon is falling.

        Args: z: Height of the balloon

        Returns: None
        """
        avg = 0
        if self.curr_pt <= 5:
            return
        for i in range(1, 5):
            avg += self.z_pts[self.curr_pt - i]
        avg = avg / 5.
        self.get_logger().info(f"Avg {avg}")
        dist = avg - z  # Previous point minus current
        if dist > 0.0:
            self.get_logger().info(f"Balloon is falling {dist}")
        return

    def depth_callback(self, depthInfo):
        """
        Obtain the frames from the depth camera.

        Args: depthInfo: Information obtained from the aligned_depth_to_color/image_raw topic

        Returns: None
        """
        try:
            self.get_logger().info("Receiving depth frames", once=True)
            curr_frame = self.bridge.imgmsg_to_cv2(depthInfo)
            self.depth_frame = curr_frame
            dpt_frame = cv.applyColorMap(cv.convertScaleAbs(self.depth_frame, alpha=0.3),
                                         cv.COLORMAP_JET)
            self.vid_pub.publish(self.bridge.cv2_to_imgmsg(dpt_frame))

        except CvBridgeError as e:
            self.get_logger().error(e)
            return

        cv.waitKey(1)

    def intr_callback(self, cameraInfo):
        """
        Get the camera information of the depth frame.

        Args: cameraInfo: Camera information obtained from the aligned_depth_to_color/camera_info
                          topic

        Returns: None
        """
        try:
            if self.intr:
                return
            self.intr = rs.intrinsics()
            self.intr.width = cameraInfo.width
            self.intr.height = cameraInfo.height
            self.intr.ppx = cameraInfo.k[2]
            self.intr.ppy = cameraInfo.k[5]
            self.intr.fx = cameraInfo.k[0]
            self.intr.fy = cameraInfo.k[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intr.model = rs.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intr.model = rs.distortion.kannala_brandt4
            self.intr.coeffs = [i for i in cameraInfo.d]
        except CvBridgeError as e:
            self.get_logger().error(e)
            return

    def conv_to_real_coords(self, x, y, depth):
        """
        Convert coordinates into real coordinates using rs2_deproject_pixel_to_point.

        Args: x, y: Coordinates of centroid read from camera
              depth: Depth calculated using the aligned_depth_to_color

        Returns: None
        """
        try:
            if self.intr:
                real_coords = rs.rs2_deproject_pixel_to_point(self.intr, [x, y], depth)
                x_ = real_coords[0] * 0.001
                y_ = real_coords[1] * 0.001
                z_ = real_coords[2] * 0.001
                return x_, y_, z_
        except CvBridgeError as e:
            self.get_logger().error(e)
            return

    def bg_sub(self, frame):
        """
        Perform background subtraction on the passed in frame, else does so on the color_frame.

        This prevents an error from occuring if there is no moving object in the scene.

        Args: frame: Video frames to perform background subtraction on

        Returns: None
        """
        if self.intr:
            mask = self.bgsub.apply(frame)
            _, mask = cv.threshold(mask, 250, 255, cv.THRESH_BINARY)
            mask = cv.morphologyEx(mask, cv.MORPH_OPEN, self.kernel)
            self.fgmask = mask
        else:
            mask = self.bgsub.apply(self.color_frame)
            _, mask = cv.threshold(mask, 250, 255, cv.THRESH_BINARY)
            mask = cv.morphologyEx(mask, cv.MORPH_OPEN, self.kernel)
            self.fgmask = mask
        return

    def remove_bg(self, clip_dist):
        """
        Remove the background using a clipping distance.

        Args: clip_dist: a distance in meters to clip to

        Returns: None
        """
        if self.intr:
            grey = 153
            clip = clip_dist * 1000
            dpt_cpy = np.asanyarray(self.depth_frame)
            clr_cpy = np.asanyarray(self.color_frame)
            depth_frame_3d = np.dstack((dpt_cpy, dpt_cpy, dpt_cpy))
            bg_removed = np.where((depth_frame_3d > clip) |
                                  (depth_frame_3d <= 0), grey, clr_cpy)
            self.bg_removed = bg_removed

    def reduce_noise(self, curr_frame, fgmask):
        """
        Use fastNlMeansDenoisingColored to reduce noise within the current frame.

        Args: curr_frame: The current frame
              fgmask: Masked image

        Returns: noiseless_curr_frame: The current frame with reduced noise
        """
        noiseless_mask = cv.fastNlMeansDenoising(fgmask, None, 20, 7, 21)
        fgmask = noiseless_mask
        noiseless_curr_frame = cv.fastNlMeansDenoisingColored(curr_frame, None, 20, 20, 7, 21)
        return noiseless_curr_frame


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

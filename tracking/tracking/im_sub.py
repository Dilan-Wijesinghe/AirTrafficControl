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


# Defining Values needed for HSV detection
# [trackbar]
# max_value = 360//2
# low_h = 100
# high_h = 360//2
# low_s = 0
# high_s = 255
# low_v = 0
# high_v = 255
# wind_name = 'Balloon Color Tracking'

# def low_h_trackbar(val):
#     global low_h
#     global high_h
#     low_h = val
#     low_h = min(high_h - 1, low_h)
#     cv.setTrackbarPos('Low H', wind_name, low_h)

# def high_h_trackbar(val):
#     global low_h
#     global high_h
#     high_h = val
#     high_h = max(high_h, low_h + 1)
#     cv.setTrackbarPos('High H', wind_name, high_h)

# def low_s_trackbar(val):
#     global low_s
#     global high_s
#     low_s = val
#     low_s = min(high_s-1, low_s)
#     cv.setTrackbarPos('Low S', wind_name, low_s)

# def high_s_trackbar(val):
#     global low_s
#     global high_s
#     high_s = val
#     high_s = max(high_s, low_s+1)
#     cv.setTrackbarPos('High S', wind_name, high_s)

# def low_v_trackbar(val):
#     global low_v
#     global high_v
#     low_v = val
#     low_v = min(high_v-1, low_v)
#     cv.setTrackbarPos('Low V', wind_name, low_v)

# def high_v_trackbar(val):
#     global low_v
#     global high_v
#     high_v = val
#     high_v = max(high_v, low_v+1)
#     cv.setTrackbarPos('High V', wind_name, high_v)

# cv.namedWindow(wind_name )
# cv.createTrackbar('Low H', wind_name , low_h, 360//2, low_h_trackbar)
# cv.createTrackbar('High H', wind_name , high_h, 360//2, high_h_trackbar)
# cv.createTrackbar('Low S', wind_name , low_s, 360//2, low_s_trackbar)
# cv.createTrackbar('High S', wind_name , high_s, max_value, high_s_trackbar)
# cv.createTrackbar('Low V', wind_name , low_v, max_value, low_v_trackbar)
# cv.createTrackbar('High V', wind_name , high_v, max_value, high_v_trackbar)
# [trackbar]


# Get the number of cameras available
def count_cameras():
    max_tested = 100
    for i in range(max_tested):
        temp_camera = cv.VideoCapture(i)
        if temp_camera.isOpened():
            temp_camera.release()
            continue
        return i
    

class State(Enum):
    """
    Current state of the robot.
    """

    NOTPUB = auto()
    PUB = auto()

"""
class ImageSubscriber

Ros2 node that subscribes to certain topics relevant to the camera node. 
Launch the camera node with:
`ros2 launch realsense2_camera \
 rs_launch.py depth_module.profile:=1280x720x3 align_depth.enable_depth:=true`

Class also implements OpenCV modifications to the incoming camera feed
"""
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        # Creating the Subscriber, which receives info from the video_frames topic
        # self.vid_sub = self.create_subscription(Image, 'video_frames', self.sub_callback, 10)
        self.vid_sub = self.create_subscription(Image, 'camera/color/image_raw', \
                                                self.sub_callback, 10)
        self.depth_sub = self.create_subscription(Image, 'camera/aligned_depth_to_color/image_raw', \
                                                  self.depth_callback, 10)
        self.intr_sub = self.create_subscription(CameraInfo, 'camera/aligned_depth_to_color/camera_info', \
                                                 self.intr_callback, 10)
        self.coords_pub = self.create_publisher(Point, 'balloon_coords', 10)

        # Publisher that publishes to the video_frames topic
        self.vid_pub = self.create_publisher(Image, 'video_frames', 10)
    
        # Create a VideoCapture object 
        # The argument '0' gives the default webcam
        # self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge() # Make a bridge to go between ROS and OpenCV Images

        # Count the number of cameras
        self.get_logger().info(f"Number of Cameras is: {count_cameras()}")

        self.kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(3,3))
        # History -> How long to keep the frames (num iterations of loop)
        # varThreshold -> How sensitive to look at the foreground objects. 
        #              -> Low varThreshold is more sensitive, high is less
        # detectShadows-> Boolean determining whether to detect shadows
        self.bgsub = cv.createBackgroundSubtractorMOG2(history = 500, varThreshold = 250, \
                                                       detectShadows = False)
        self.state = State.NOTPUB
        
        # Frame Instantiation
        self.intr = None
        self.color_frame = None
        self.depth_frame = None
        self.fgmask = None
        self.bg_removed = None

    """ 
    TODO
    # Find Threshold Range
            # Sample multiple right pixels (from HSV)
        # Denoise
            # Get rid of chatter and outliers
        # Depth Camera
            # Get rid of something that is very far away 
            # Background Subtraction is similar 
    """

    """
    sub_callback()

    Subscription to the colored raw image coming in from the camera
    Performs several OpenCV operations on the frame
    """
    def sub_callback(self, data):
        try:
            self.get_logger().info("Receiving video_frames", once=True)
            # curr_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='mono8') # Convert ROS image msg to OpenCV image
            curr_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8') # Convert ROS image msg to OpenCV image
            self.color_frame = cv.GaussianBlur(curr_frame, (5,5), 0)
            self.vid_pub.publish(self.bridge.cv2_to_imgmsg(self.color_frame)) # Publish msg 

            self.remove_bg(clip_dist=2) # Remove background
            self.bg_sub(self.bg_removed) # Apply Background subtraction

            # Detect Contours
            contours, _ = cv.findContours(self.fgmask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            self.coords = Point()

           
            try:
                # print("Found Contours")
                areas = [cv.contourArea(cont) for cont in contours] # Loop over the contours
                MaxCont = contours[np.argmax(areas)] # And find areas. Then find Max
            
            except:
                print("No Contours") 

            centroids = [] 
            try:
                # print(self.MaxCentroid)
                # --- Drawing Onto the Frames ---
                ((x,y), r) = cv.minEnclosingCircle(MaxCont)
                if self.intr:
                    if r > 50:
                        self.state = State.PUB # Change ability to publish
                        print("Found An Object to Publish")
                        # --- Current Moment for Centroid Finding ---
                        for cont in contours:
                            # if cv.contourArea(cont) > 1000:
                            currMoment = cv.moments(cont)
                            try:
                                cx = int(currMoment['m10']/currMoment['m00'])
                                cy = int(currMoment['m01']/currMoment['m00'])
                                # print("Adding Centroid")
                                centroids.append([cx, cy])
                            except:
                                print("Centroid Not Found")
                        self.MaxCentroid = centroids[np.argmax(areas)]
                        # x, y, w, h = cv.boundingRect(MaxCont) # Draw a Bounding Shape around the Max Contour
                        # cv.rectangle(self.color_frame, (x,y), (x + w, y + h), (0, 0, 255), 2) # Draw rect over obj
                        cv.circle(self.color_frame, (int(x), int(y)), int(r), (0, 255, 255), 2)
                        cv.circle(self.color_frame, self.MaxCentroid, 10, (24,146,221), -1) # Thickness of -1 Fills in 
                        cv.putText(self.color_frame, "Found Object", (x,y-10), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0,255,0),1, cv.LINE_AA)
                    else: # r < 50
                        self.state = State.NOTPUB # If it is too small, do not publish
            except:
                pass

            # Get Real Moving Object
            real = cv.bitwise_and(self.color_frame, self.color_frame, mask=self.fgmask)
            #real = cv.bitwise_and(self.bg_removed, self.bg_removed, mask=self.fgmask)

            # fgmask 3 channeled
            fgmask_3 = cv.cvtColor(self.fgmask, cv.COLOR_GRAY2BGR)

            # Stack all three frames 
            stacked = np.vstack((fgmask_3, self.color_frame, real))
            cv.imshow("Stacked", cv.resize(stacked, None, fx=0.40, fy=0.40))

            # --- Get Real Coords ---
            if len(areas) != 0:
                try:
                    if self.intr:
                        depth = self.depth_frame[self.MaxCentroid[1], self.MaxCentroid[0]]
                        # depth = self.depth_frame[self.MaxCentroid[0], self.MaxCentroid[1]]
                        x_, y_, z_ = self.conv_to_real_coords(self.MaxCentroid[0], \
                                                              self.MaxCentroid[1], depth)
                        self.coords.x = x_
                        self.coords.y = y_
                        self.coords.z = z_

                        if self.state == State.PUB:
                            self.get_logger().info(f"Real Coords are: \
                                {self.coords.x, self.coords.y, self.coords.z}")
                            self.coords_pub.publish(self.coords) # Publishing coords
                except:
                    pass
            else:
                print("Cannot Calculate Centroid Depth")

            # cv.imshow('Background Sub', fgmask)
            cv.waitKey(1)
            
        except CvBridgeError as e:
            self.get_logger().error(e)
            return

        cv.waitKey(1)
     
    
    def depth_callback(self, depthInfo):
        """
        Callback function that obtains the frames from the depth camera

        Args: depthInfo : Information obtained from the aligned_depth_to_color/image_raw topic

        Returns: None
        """
        try:
            self.get_logger().info("Receiving depth frames", once=True)
            curr_frame = self.bridge.imgmsg_to_cv2(depthInfo)
            self.depth_frame = curr_frame
            dpt_frame = cv.applyColorMap(cv.convertScaleAbs(self.depth_frame, alpha=0.3), \
                                         cv.COLORMAP_JET)
            self.vid_pub.publish(self.bridge.cv2_to_imgmsg(dpt_frame))

        except CvBridgeError as e:
            self.get_logger().error(e)
            return

        # cv.imshow("Depth", dpt_frame) # Display depth image    
        cv.waitKey(1)

    def intr_callback(self, cameraInfo):
        """
        Instrinsic Callback function. Gets the camera information of the Depth frame
        In order to apply the rs2_deproject_pixel_to_point function

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
        Converts coordinates into real coordinates using rs2_deproject_pixel_to_point
        
        Args: x, y : Coords of Centroid read from camera
              depth : Depth calculated using the aligned_depth_to_color

        Returns: None
        """
        try:
            if self.intr:
                real_coords = rs.rs2_deproject_pixel_to_point(self.intr, [x,y], depth)
                # x_ = real_coords[2] * 0.001
                # y_ = real_coords[0] * 0.001
                # z_ = real_coords[1] * 0.001
                x_ = real_coords[0] * 0.001
                y_ = real_coords[1] * 0.001
                z_ = real_coords[2] * 0.001
                # self.get_logger().info(f"Real Coords are: {x_, y_, z_}")
                return x_,y_,z_
        except CvBridgeError as e:
            self.get_logger().error(e)
            return
    
    def bg_sub(self, frame):
        """
        Performs Background Subtraction on the passed in frame, else
        does so on the color_frame. This prevents an error from occuring 
        if there is no moving object in the scene

        Args: frame : video frames to perform background subtraction on

        Returns: None 
        """
        if self.intr:
            mask = self.bgsub.apply(frame)
            _, mask = cv.threshold(mask, 250, 255, cv.THRESH_BINARY)
            mask = cv.morphologyEx(mask, cv.MORPH_OPEN, self.kernel)
            self.fgmask = mask 
        # fgmask = cv.erode(fgmask, kernel=self.kernel, iterations = 1)
        # fgmask = cv.dilate(fgmask, kernel=self.kernel, iterations = 2)
        else: 
            mask = self.bgsub.apply(self.color_frame)
            _, mask = cv.threshold(mask, 250, 255, cv.THRESH_BINARY)
            mask = cv.morphologyEx(mask, cv.MORPH_OPEN, self.kernel)
            self.fgmask = mask 
        return

    
    def remove_bg(self, clip_dist):
        """
        Removes the Background using a clipping distance

        Args: clip_dist : a distance in meters to clip to

        Returns: None
        """
        if self.intr:
            grey = 153
            clip = clip_dist * 1000
            dpt_cpy = np.asanyarray(self.depth_frame)
            clr_cpy = np.asanyarray(self.color_frame)
            depth_frame_3d = np.dstack((dpt_cpy, dpt_cpy, dpt_cpy))
            bg_removed = np.where((depth_frame_3d > clip) | \
                                  (depth_frame_3d <= 0), grey, clr_cpy)
            self.bg_removed = bg_removed
            
            # ValueError: operands could not be broadcast together with shapes (480,848,3) () (720,1280,3) 
    

    def reduce_noise(self, curr_frame, fgmask):
        """
        Helper function that uses fastNlMeansDenoisingColored to reduce noise within the current frame

        Input - curr_frame - The current frame
                - fgmask - Masked image
        Returns - noiseless_curr_frame - The current frame with reduced noise
        """
        # Need some noise reduction here
        noiseless_mask = cv.fastNlMeansDenoising(fgmask, None, 20, 7, 21) 
        fgmask = noiseless_mask
        noiseless_curr_frame = cv.fastNlMeansDenoisingColored(curr_frame ,None,20,20,7,21) 
        return noiseless_curr_frame


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# Contours
# contours, hier = cv.findContours(frame_thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

# --- Centroids and Areas ---
# centroids = []
# areas = []
# for cont in contours:
#     currMom = cv.moments(cont)
#     cont_area = cv.contourArea(cont)
#     try: 
#         cx = int(currMom['m10']/currMom['m00'])
#         cy = int(currMom['m01']/currMom['m00'])
#         centroids.append([cx, cy])# gray = cv.cvtColor(curr_frame, cv.COLOR_BGR2GRAY)
# frame_HSV = cv.cvtColor(curr_frame, cv.COLOR_BGR2HSV)
# frame_thresh = cv.inRange(frame_HSV, (low_h, low_s, low_v), (high_h, high_s, high_v))
# cv.imshow(wind_name, frame_thresh)

# Contours
# contours, hier = cv.findContours(frame_thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

# --- Centroids and Areas ---
#         pass

# try: 
#     # Find Largest Contour
#     MaxCont = np.argmax(areas)
#     MaxCentroid = centroids[MaxCont]
#     curr_frame = cv.drawContours(curr_frame, contours, -1, (255,204,0))
#     curr_frame = cv.circle(curr_frame, MaxCentroid, 10, (24, 146, 221))
#     print(f"Centroid: {MaxCentroid}")
# except:
#     print("Contour not found")



# --- Find Canny Edges ---
# edged = cv.Canny(fgmask, 30, 200)
# # edged = cv.Canny(curr_frame, 30, 200)
# contours, hier = cv.findContours(edged, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
# cv.imshow('Canny Edges', edged)

# cv.drawContours(curr_frame, contours, -1, (0, 255, 0), 3)

# --- Hough Circle ---
# param1 - sensitivity
# param2 - number of edgepoints
# circles = cv.HoughCircles(curr_frame, cv.HOUGH_GRADIENT, 1, 1000,
#                           param1=150, param2=60, minRadius=0, maxRadius=150)

# if circles is None:
#     print("No circles!")
#     pass
# else:
#     print(circles)
#     circles = np.uint16(np.around(circles))

#     for i in circles[0,:]:
#         cv.circle(curr_frame, (i[0], i[1]), i[2], (0,255,0), 2)
#         cv.circle(curr_frame, (i[0], i[1]), 2, (0, 0, 255), 3)


# cv.imshow('Contours', curr_frame)
# cv.imshow("Camera + Contours", frame_HSV) # Display image
# cv.imshow("FG Mask", fgmask) # Display foreground mask




# --- Color Variations ---
# gray = cv.cvtColor(curr_frame, cv.COLOR_BGR2GRAY)
# frame_HSV = cv.cvtColor(curr_frame, cv.COLOR_BGR2HSV)
# frame_thresh = cv.inRange(frame_HSV, (low_h, low_s, low_v), (high_h, high_s, high_v))
# cv.imshow(wind_name, frame_thresh)
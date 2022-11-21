# Imports 
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

max_value = 360//2
low_h = 100
high_h = 360//2
low_s = 0
high_s = 255
low_v = 0
high_v = 255
wind_name = 'Balloon Color Tracking'

def low_h_trackbar(val):
    global low_h
    global high_h
    low_h = val
    low_h = min(high_h - 1, low_h)
    cv.setTrackbarPos('Low H', wind_name, low_h)

def high_h_trackbar(val):
    global low_h
    global high_h
    high_h = val
    high_h = max(high_h, low_h + 1)
    cv.setTrackbarPos('High H', wind_name, high_h)

def low_s_trackbar(val):
    global low_s
    global high_s
    low_s = val
    low_s = min(high_s-1, low_s)
    cv.setTrackbarPos('Low S', wind_name, low_s)

def high_s_trackbar(val):
    global low_s
    global high_s
    high_s = val
    high_s = max(high_s, low_s+1)
    cv.setTrackbarPos('High S', wind_name, high_s)

def low_v_trackbar(val):
    global low_v
    global high_v
    low_v = val
    low_v = min(high_v-1, low_v)
    cv.setTrackbarPos('Low V', wind_name, low_v)

def high_v_trackbar(val):
    global low_v
    global high_v
    high_v = val
    high_v = max(high_v, low_v+1)
    cv.setTrackbarPos('High V', wind_name, high_v)

cv.namedWindow(wind_name )
cv.createTrackbar('Low H', wind_name , low_h, 360//2, low_h_trackbar)
cv.createTrackbar('High H', wind_name , high_h, 360//2, high_h_trackbar)
cv.createTrackbar('Low S', wind_name , low_s, 360//2, low_s_trackbar)
cv.createTrackbar('High S', wind_name , high_s, max_value, high_s_trackbar)
cv.createTrackbar('Low V', wind_name , low_v, max_value, low_v_trackbar)
cv.createTrackbar('High V', wind_name , high_v, max_value, high_v_trackbar)
## [trackbar]

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
        # History -> How long to keep the frames (num iterations of loop)
        # varThreshold -> How sensitive to look at the foreground objects. 
        #              -> Low varThreshold is more sensitive, high is less
        self.bgsub = cv.createBackgroundSubtractorMOG2(history = 1000, varThreshold = 1000, detectShadows=False)

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
        # curr_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='mono8') # Convert ROS image msg to OpenCV image
        curr_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8') # Convert ROS image msg to OpenCV image
        # gray = cv.cvtColor(curr_frame, cv.COLOR_BGR2GRAY)
        # frame_HSV = cv.cvtColor(curr_frame, cv.COLOR_BGR2HSV)
        # frame_thresh = cv.inRange(frame_HSV, (low_h, low_s, low_v), (high_h, high_s, high_v))
        # cv.imshow(wind_name, frame_thresh)

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
        #         centroids.append([cx, cy])
        #         areas.append(cont_area)

        #     except:
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

        # Find Threshold Range
            # Sample multiple right pixels (from HSV)
        # Denoise
            # Get rid of chatter and outliers
        # Depth Camera
            # Get rid of something that is very far away 
            # Background Subtraction is similar 

        # Background Subtraction
        # im_rgb = cv2.cvtColor(curr_frame, cv2.COLOR_RGB2BGR)

        # Apply the Background Subtraction
        fgmask = self.bgsub.apply(curr_frame)
        _, fgmask = cv.threshold(fgmask, 250, 255, cv.THRESH_BINARY)
        fgmask = cv.morphologyEx(fgmask, cv.MORPH_OPEN, self.kernel) # Morph ops?
        fgmask = cv.erode(fgmask, kernel=self.kernel, iterations = 1)
        fgmask = cv.dilate(fgmask, kernel=self.kernel, iterations = 2)

        # Need some noise reduction here

        # Detect Contours
        contours, _ = cv.findContours(fgmask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        # Loop over the contours
        try:
            print("Found Contours")
            areas = [cv.contourArea(cont) for cont in contours]
            MaxCont = contours[np.argmax(areas)] 
            # for cont in contours:
                # if cv.contourArea(cont) > 1000:
            x, y, w, h = cv.boundingRect(MaxCont)
            cv.rectangle(curr_frame, (x,y), (x + w, y + h), (0, 0, 255), 2) # Draw rect over obj
            cv.putText(curr_frame, "Found Object", (x,y-10), cv.FONT_HERSHEY_SIMPLEX, 0.3, (0,255,0),1, cv.LINE_AA)
        except:
            print("No Contours")

        # Get Real Moving Object
        real = cv.bitwise_and(curr_frame, curr_frame, mask=fgmask)

        # fgmask 3 channeled
        fgmask_3 = cv.cvtColor(fgmask, cv.COLOR_GRAY2BGR)

        # Stack all three frames 
        stacked = np.vstack((fgmask_3, curr_frame, real))
        cv.imshow("Stacked", cv.resize(stacked, None, fx=0.40, fy=0.40))
       
        # cv.imshow('Background Sub', fgmask)

        # Find Canny Edges
        # edged = cv.Canny(fgmask, 30, 200)
        # # edged = cv.Canny(curr_frame, 30, 200)
        # contours, hier = cv.findContours(edged, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        # cv.imshow('Canny Edges', edged)

        # cv.drawContours(curr_frame, contours, -1, (0, 255, 0), 3)

        # Hough Circle 
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
        
        self.vid_pub.publish(self.bridge.cv2_to_imgmsg(curr_frame)) # Publish msg 
        # self.get_logger().info(self.bridge.cv_to_imgmsg(im_rgb), once=True)

        # cv.imshow('Contours', curr_frame)
        # cv.imshow("Camera + Contours", frame_HSV) # Display image
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



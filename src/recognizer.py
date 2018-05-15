import cv2
import rospkg
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
rospack = rospkg.RosPack()
imagesPath = rospack.get_path('my_gui') + '/src/images/'

class Recognizer:
    def imgCallback(self, data):
        self.image = data

    def __init__(self):
        self.image = None
        rospy.Subscriber('/art/localhost/kinect2/hd/image_color_rect', Image, self.imgCallback)
        
    def getDicesValue(self):
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")
        
        frame = frame[300:500, 700:1000]
        
        r = 500.0 / frame.shape[1]
        dim = dim = (500, int(frame.shape[0] * r))
        frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        ret,frame = cv2.threshold(frame, 130,255,cv2.THRESH_BINARY)

        height, width = frame.shape[:2]
        cv2.floodFill(frame, None, (0,0), 255)
        cv2.floodFill(frame, None, (0,height-1), 255)
        cv2.floodFill(frame, None, (width-1,0), 255)
        cv2.floodFill(frame, None, (width-1,height-1), 255)
        cv2.floodFill(frame, None, (0,height/2), 255)

        params = cv2.SimpleBlobDetector_Params()
        params.minDistBetweenBlobs = 0
        params.filterByArea = True
        params.minArea = 1
        params.filterByArea = True
        params.minArea = 70
        params.filterByInertia = True
        params.minInertiaRatio = 0.5
        params.minThreshold = 10;
        params.maxThreshold = 200;

        detector = cv2.SimpleBlobDetector_create(params)
    
        keypoints = detector.detect(frame)
        frame = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        return len(keypoints)
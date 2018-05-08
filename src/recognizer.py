import cv2
import rospkg
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

rospack = rospkg.RosPack()
imagesPath = rospack.get_path('my_gui') + '/src/images/'

class Recognizer:
    def __init__(self):
        #rospy.Subscriber('', Image, self.image)
        pass
        
    def getDicesValue(self):
        # bridge = CvBridge()
        # frame = bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")
        frame = cv2.imread(imagesPath + "photo4-dices.jpg")
        # cv2.imshow("image", frame)
        # cv2.waitKey(0)

        # frame = frame[1900:2200, 1200:1450]
        # cv2.imshow("cropped", frame)
        # cv2.waitKey(0)

        r = 400.0 / frame.shape[1]
        dim = dim = (400, int(frame.shape[0] * r))
        frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
        cv2.imshow("resized", frame)
        cv2.waitKey(0)

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # cv2.imshow("grayscale", frame)
        # cv2.waitKey(0)

        ret,frame = cv2.threshold(frame, 30,255,cv2.THRESH_BINARY)
        # cv2.imshow("threshold", frame)
        # cv2.waitKey(0)

        # frame = frame
        # frame = cv2.Canny(frame,100,200)
        # cv2.imshow("edges", frame)
        # cv2.waitKey(0)
        cv2.floodFill(frame, None, (0,0), 255);
        cv2.imshow("floodfill", frame)
        cv2.waitKey(0)

        params = cv2.SimpleBlobDetector_Params()
        params.minDistBetweenBlobs = 0
        params.filterByArea = True
        params.minArea = 1
        params.minThreshold = 10;
        params.maxThreshold = 200;
        detector = cv2.SimpleBlobDetector(params)
        keypoints = detector.detect(frame)
        frame = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        print(len(keypoints))
        cv2.imshow("blobs", frame)
        cv2.waitKey(0)

        # contours, hierarchy = cv2.findContours(frame,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(frame, contours, -1, (0,255,0), 3)
        # cv2.imshow("contours", frame)
        # cv2.waitKey(0)
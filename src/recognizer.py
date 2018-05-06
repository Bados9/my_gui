import cv2
import rospkg
import rospy
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
        frame = cv2.imread(imagesPath + "bp_recognizing_test.jpg")
        # cv2.imshow("image", frame)
        # cv2.waitKey(0)

        gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # cv2.imshow("grayscale", gray_img)
        # cv2.waitKey(0)

        ret,thresh = cv2.threshold(gray_img,150,255,cv2.THRESH_BINARY)
        cv2.imshow("threshold", thresh)
        cv2.waitKey(0)

        crop_img = thresh[180:220, 120:150]
        cv2.imshow("cropped", crop_img)
        cv2.waitKey(0)

        r = 200.0 / crop_img.shape[1]
        dim = dim = (200, int(crop_img.shape[0] * r))
        resized_img = cv2.resize(crop_img, dim, interpolation = cv2.INTER_AREA)
        cv2.imshow("resized", resized_img)
        cv2.waitKey(0)

        edges_img = resized_img
        #edges_img = cv2.Canny(resized_img,100,200)
        cv2.imshow("edges", edges_img)
        cv2.waitKey(0)

        contours, hierarchy = cv2.findContours(edges_img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(edges_img, contours, -1, (0,255,0), 3)
        cv2.imshow("contours", edges_img)
        cv2.waitKey(0)
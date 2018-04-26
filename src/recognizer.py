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
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")
        #frame = cv2.imread(imagesPath + "cat.jpeg")
        cv2.imshow("image", frame)
        cv2.waitKey(0)

        crop_img = frame[0:200, 50:100]
        cv2.imshow("cropped", crop_img)
        cv2.waitKey(0)

        
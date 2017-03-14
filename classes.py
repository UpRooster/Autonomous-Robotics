# Encoding: utf-8
# LTB - 14475946
# CV.py
"""
Created on Tues Mar 14 13:11

@author: Liam T Berridge
"""

# Imports
import rospy, cv2, numpy

#Import Definitions
# cv2 Imports
from cv2 import namedWindow, cvtColor, imshow
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY
from cv2 import blur, Canny
# from numpy import mean
# ROSpy Imports
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class Vision:

    def __init__(self):

        # Init dimension Variables
        self.h = 0
        self.w = 0
        self.d = 0

        # Init camera output windows
        namedWindow("Grayscale",1)
        namedWindow("Blur",1)
        namedWindow("Canny Edge",1)
        namedWindow("Canny EdgeB",1)
        namedWindow("Target Range",1)
        namedWindow("Mask",1)

        # Bridge ROSpy & cv2
        self.bridge = CvBridge()

        # Init window threads
        startWindowThread()

        # Init subscribers & image proc
        try:
            self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.visioncall)
        except:
            print "No NetTurtle Found!"
        try:
            self.image_sub = rospy.Subscriber("/turtlebot_1/camera/rgb/image_raw",Image,self.visioncall)
        except:
            print "No SimTurtle Found Either!"
            return "No Turtles! :("

    def visioncall(self, data):
        # Image ROSpy to cv2 data
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # Img to grayscale
        grayImg = cvtColor(cv_image, COLOR_BGR2GRAY)
        # print mean(gray_img) # WARNING CONSOLE SPAM

        # Apply Filters and Output Images to Windows
        imgBlur = blur(grayImg, (3,3))
        imgCanny = Canny(grayImg, 10, 200)
        imgCannyB = Canny(imgBlur, 10, 200)

        imshow("Grayscale", grayImg)
        imshow("Blur", imgBlur)
        imshow("Canny Edge", imgCanny)
        imshow("Canny EdgeB", imgCannyB)

        # Colour Detection (BGR) DETECTS RED/BROWN
        lowThresh = numpy.array([0,100,0])
        highThresh = numpy.array([130,255,130])

        imgRange = cv2.inRange(cv_image, lowThresh, highThresh)
        imshow("Target Range", imgRange)

        # Define input & masking dimensions
        h, w, d = cv_image.shape

        search_top = 3*h/4
        search_bot = (3*h/4)+(1*h/4)

        imgRange[0:search_top, 0:w] = 0
        imgRange[search_bot:0, 0:w] = 0

        imshow("Mask", imgRange)
        # Define object in view
        M = cv2.moments(imgRange)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(cv_image, (cx, cy), 20, (0,0,255), -1)
            
rospy.init_node('vision')
vision = Vision()
rospy.spin()

#class mapping:
#class motion:

# -*- coding: utf-8 -*-
"""
Created on Mon Mar 20 20:32:08 2017

@author: computing
"""

# -*- coding: utf-8 -*-
"""
Created on Thu Feb  9 16:27:03 2017

@author: computing
"""
#!/usr/bin/env python

# rospy - ROS Python Core
import rospy
# cv2	- Camera View suite
import cv2
# numpy	- Numerical Python suite --Allows array objects
import numpy

# cv2	- Import existing definitions
from cv2 import *

# numpy	- Impoty mean definition
from numpy import mean, nanmin

# ROS imports
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class robotSeeker:
#=========================================================================================================================
    def __init__(self):
        # Define Global Variables               ========
        self.midLaser = 999
        self.colourMatch = 0
        self.colourFound = [0,0,0,0]
        self.roamCo = 0
        self.M = []
        # Functions                             ========
        self.bridge = CvBridge()
        startWindowThread()
        # Subscribers                           ========
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw",Image,self.callback)
        self.laser = rospy.Subscriber("/turtlebot/scan", LaserScan, self.laser_callback)
        # Publishers                            ========
        self.twist_pub = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size=1)
        self.T = Twist()
        # Remove previous movement commands
        self.T.angular.z = 0
        self.T.linear.x = 0
        self.twist_pub.publish(self.T)
        # Create image output windows           ========
        namedWindow("Image window", 1)
        namedWindow("Mask", 1)
#=========================================================================================================================
    def callback(self, data):
        # Get Raw Camera Feed & Convert to HSV
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsvImg = cvtColor(cv_image, COLOR_BGR2HSV)
        # Get colour defintions
        self.colour_select()
        # Allowable Range (Detects colours inside threshold)   ========
        maskImg = cv2.inRange(hsvImg, self.lowerThresh, self.highrThresh)
        h, w, d = cv_image.shape
        search_top = h/4
        search_bot = 3*h/4 + 20
        maskImg[0:search_top, 0:w] = 0
        maskImg[search_bot:h, 0:w] = 0      
        imshow("Image window", cv_image)
        imshow("Mask", maskImg)

        self.M = cv2.moments(maskImg)
        if nanmin(self.laserArray,axis = 0) >= 1:
            if self.M['m00'] > 100:
                cx = int(self.M['m10']/self.M['m00'])
                cy = int(self.M['m01']/self.M['m00'])
                cv2.circle(cv_image, (cx, cy), 20, (0,0,255), -1)
                err = cx - w/2
                self.T.angular.z = -float(err) / 100
                self.move_seek()
            else:
                self.colourMatch = (self.colourMatch+1)%4
                self.move_roam()
        else:
            self.avoidance()
        self.twist_pub.publish(self.T)
        if (self.colourFound == [1, 1, 1, 1]):
            print "All Colours Found!"
            self.T.angular.z = 0.3
            self.twist_pub.publish(self.T)
#=========================================================================================================================
    def laser_callback(self,msg):
        self.midLaser = msg.ranges[len(msg.ranges)/2]
        self.laserArray = msg.ranges
#=========================================================================================================================
    def colour_select(self):
                # Define Colours for Search & Set Search Triggers   ========
        if (self.colourMatch == 0 and self.colourFound[0] == 0): # Green
            self.lowerThresh = numpy.array([50,200,100])
            self.highrThresh = numpy.array([100,255,250])
        elif (self.colourMatch == 1 and self.colourFound[1] == 0): # Blue
            self.lowerThresh = numpy.array([100,200,100])
            self.highrThresh = numpy.array([150,255,255])
        elif (self.colourMatch == 2 and self.colourFound[2] == 0): # Yellow
            self.lowerThresh = numpy.array([30,200,100])
            self.highrThresh = numpy.array([50,255,150])
        elif (self.colourMatch == 3 and self.colourFound[3] == 0): # Red
            self.lowerThresh = numpy.array([0,200,100])
            self.highrThresh = numpy.array([0,255,150])
        else:
            self.lowerThresh = numpy.array([0,0,0]) # Black Mask
            self.highrThresh = numpy.array([0,0,0]) # Black Mask
#=========================================================================================================================
    def colour_goals(self):
        # Create and Mark goals with tracking
        if (self.colourMatch == 0 and self.M['m00'] > 6000000):
            self.colourFound[0] = 1
            self.colourMatch = (self.colourMatch+1)%4
            print "Found Green Object!"
        elif (self.colourMatch == 1 and self.M['m00'] > 6000000):
            self.colourFound[1] = 1
            self.colourMatch = (self.colourMatch+1)%4
            print "Found Blue Object!"
        elif (self.colourMatch == 2 and self.M['m00'] > 6000000):
            self.colourFound[2] = 1
            self.colourMatch = (self.colourMatch+1)%4
            print "Found Yellow Object!"
        elif (self.colourMatch == 3 and self.M['m00'] > 6000000):
            self.colourFound[3] = 1
            self.colourMatch = (self.colourMatch+1)%4
            print "Found Red Object!"
#=========================================================================================================================
    def move_seek(self):
        #print "Seeking!"
        # Define Twist (Movement Call)          ========
        # Move to Colour!
        self.colour_goals()
        self.T.linear.x = 0.4
        self.twist_pub.publish(self.T)
#=========================================================================================================================
    def move_roam(self):
        #print "Roaming!"
        self.T.linear.x = 0.6
        self.T.angular.z = 0
        self.twist_pub.publish(self.T)
        self.roamCo = self.roamCo + 1
        if self.roamCo > 140:
            self.T.linear.x = 0
            self.T.angular.z = 1.2
        if self.roamCo > 260:
            self.roamCo = 0
            
#=========================================================================================================================
    def avoidance(self):
        #print "Too Close!"
        mid = len(self.laserArray)/2
        if nanmean(self.laserArray) < 1.75:
                self.T.angular.z = -2.4
                self.T.linear.x = 0
        elif nansum(self.laserArray[:mid-40]) < nansum(self.laserArray[mid+40:]):
            self.T.angular.z = 2
            #print "Left!"
        elif nansum(self.laserArray[:mid-40]) > nansum(self.laserArray[mid+40:]):
            self.T.angular.z = -2
            #print "Right!"
        self.twist_pub.publish(self.T)
#=========================================================================================================================            
# Init ROSPY node
rospy.init_node('robotSeeker')

# Shortened definition for class
rs = robotSeeker()

# Block interaction until node is shutdown
rospy.spin()

# Cleanup Windows
destroyAllWindows()

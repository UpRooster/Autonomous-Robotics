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
from numpy import mean

# ROS imports
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class image_converter:

   

    def __init__(self):

	# Create image output windows
        namedWindow("Image window", 1)
        namedWindow("Target Range", 1)
        namedWindow("Mask", 1)

	# Bridge ROSPY and CV2
        self.bridge = CvBridge()
	
	# Starts image thread
        startWindowThread()

        #self.image_sub = rospy.Subscriber("/video", Image, self.callback)
	
	# Subscribe to raw_img from turtlebot | call definition "callback"
        self.laserArray = []
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw",Image,self.callback)
        self.laser = rospy.Subscriber("/turtlebot/scan", LaserScan, self.laser_callback)
        self.wl = rospy.Subscriber("/turtlebot/wheel_left",  Float32)
        self.wr = rospy.Subscriber("/turtlebot/wheel_right", Float32)
        
        
        self.midLaser = 999
        self.colourMatch = 0
        self.colourFound = [0,0,0,0]
        
	# Create publishing to turtlebot (image + velocity)
        self.twist_pub = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size=1)

	# Blank variables for image processing
        self.h = 0
        self.w = 0
        self.d = 0

	# Variables for robot dimensions/manuevering
        self.wheel_radius = 1
        self.robot_radius = 1
        
        

    def callback(self, data):
        # Define Twist (Movement Call)        
        T = Twist()
        # Convert Rospy to Cv2 datatype (Get RAW Image)
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        imshow("Image window", cv_image)

        # Convert BGR to HSV values
        hsvImg = cvtColor(cv_image, COLOR_BGR2HSV)
        
        if (self.colourMatch == 0 and self.colourFound[0] == 0): # Green
            lowerThresh = numpy.array([50,200,100])
            highrThresh = numpy.array([100,255,250])
        elif (self.colourMatch == 1 and self.colourFound[1] == 0): # Blue
            lowerThresh = numpy.array([100,200,100])
            highrThresh = numpy.array([150,255,255])
        elif (self.colourMatch == 2 and self.colourFound[2] == 0): # Yellow
            lowerThresh = numpy.array([30,200,100])
            highrThresh = numpy.array([50,255,150])
        elif (self.colourMatch == 3 and self.colourFound[3] == 0): # Red
            lowerThresh = numpy.array([0,200,100])
            highrThresh = numpy.array([0,255,150])
        else:
            lowerThresh = numpy.array([0,0,0]) # Black Mask
            highrThresh = numpy.array([0,0,0]) # Black Mask
            
	# Allowable Range (Detects colours inside threshold)
        img4 = cv2.inRange(hsvImg, lowerThresh, highrThresh)
        
	# Display target range image
        imshow("Target Range", img4)
        h, w, d = cv_image.shape
        search_top = h/4
        search_bot = 3*h/4 + 20
        img4[0:search_top, 0:w] = 0
        img4[search_bot:h, 0:w] = 0
        imshow("Mask", img4)
        M = cv2.moments(img4)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(cv_image, (cx, cy), 20, (0,0,255), -1)
            # BEGIN CONTROL
            err = cx - w/2
            T.angular.z = -float(err) / 100
            if (self.laserArray[1] < 0.775):
                T.angular.z = 2
                rospy.sleep(1)
            if (self.laserArray[len(self.laserArray)-1] < 0.775):
                T.angular.z = -2
                rospy.sleep(1)
            if (self.midLaser >= 1.2) | (isnan(self.midLaser)):
                T.linear.x = 1
            if (self.midLaser < 1.2 and self.midLaser >= 0.75):
                T.linear.x = 0.4
                if (self.colourMatch == 0 and M['m00'] > 6000000):
                    self.colourFound[0] = 1
                    self.colourMatch = (self.colourMatch+1)%4
                    print "Found Green Object!"
                elif (self.colourMatch == 1 and M['m00'] > 6000000):
                    self.colourFound[1] = 1
                    self.colourMatch = (self.colourMatch+1)%4
                    print "Found Blue Object!"
                elif (self.colourMatch == 2 and M['m00'] > 6000000):
                    self.colourFound[2] = 1
                    self.colourMatch = (self.colourMatch+1)%4
                    print "Found Yellow Object!"
                elif (self.colourMatch == 3 and M['m00'] > 6000000):
                    self.colourFound[3] = 1
                    self.colourMatch = (self.colourMatch+1)%4
                    print "Found Red Object!"
            if (self.midLaser < 0.75):
                print "Too Close!"
                T.linear.x = -0.2
        else:
            self.colourMatch = (self.colourMatch+1)%4
            T.linear.x = 1
            
        self.twist_pub.publish(T)
        if (self.colourFound == [1, 1, 1, 1]):
            print "All Colours Found!"
            T.angular.z = 10
            self.twist_pub.publish(T)
        
#    # Definition for independant wheel movement
#    def forward_kinematics(self, w_l, w_r):
#	# Left Wheel
#        c_l = self.wheel_radius * w_l
#	# Right Wheel
#        c_r = self.wheel_radius * w_r
#	# Velocity calc
#        v = (c_l + c_r) / 2
#	# Acceleration calc
#        a = (c_l - c_r) / self.robot_radius
#        return (v, a)
        
    def laser_callback(self,msg):
        self.midLaser = msg.ranges[len(msg.ranges)/2]
        self.laserArray = msg.ranges
        
# Init ROSPY node
rospy.init_node('image_converter')

# Shortened definition for class
ic = image_converter()

# Block interaction until node is shutdown
rospy.spin()

# Cleanup Windows
destroyAllWindows()

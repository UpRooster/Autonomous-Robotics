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
from cv2 import namedWindow, cvtColor, imshow
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY
from cv2 import blur, Canny

# numpy	- Impoty mean definition
# from numpy import mean

# ROS imports
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class image_converter:

   

    def __init__(self):

	# Create image output windows
        namedWindow("Image window", 1)
        
	# DEBUG - DEBUG WINDOWS
        namedWindow("blur", 1)
        namedWindow("canny", 1)

        namedWindow("Target Range", 1)
        namedWindow("Mask", 1)

	# Bridge ROSPY and CV2
        self.bridge = CvBridge()
	
	# Starts image thread
        startWindowThread()

        #self.image_sub = rospy.Subscriber("/video", Image, self.callback)
	
	# Subscribe to raw_img from turtlebot | call definition "callback"
        self.image_sub = rospy.Subscriber("/turtlebot_1/camera/rgb/image_raw",Image,self.callback)
        self.wl = rospy.Subscriber("/turtlebot_1/wheel_left",  Float32)
        self.wr = rospy.Subscriber("/turtlebot_1/wheel_right", Float32)         
        
	# Create publishing to turtlebot (image + velocity)
        self.image_pub = rospy.Publisher('/image/GeorgesAmazingMonster', Image, latch = True)
        self.twist_pub = rospy.Publisher('/turtlebot_1/cmd_vel', Twist)

	# Blank variables for image processing
        self.h = 0
        self.w = 0
        self.d = 0

	# Variables for robot dimensions/manuevering
        self.wheel_radius = 1
        self.robot_radius = 1
        
        

    def callback(self, data):
	# cv_image bridges from ROSPY to CV2 using "data" and BGR2 format
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

	# Convert cv_image to grayscale
        gray_img = cvtColor(cv_image, COLOR_BGR2GRAY)
        #print mean(gray_img)

	# Blur/Edge (used for defining world inside image)
        img2 = blur(gray_img, (3, 3))
	# DEBUG -- Show blur
        imshow("blur", img2)
        img3 = Canny(gray_img, 10, 200)
        # DEBUG -- Show canny
        imshow("canny", img3)
	# DEBUG -- Show grayscale
        imshow("Image window", gray_img)

	# Create TARGET thresholds for detection (GREEN)
	# Lower Threshold (Dark Green)
        lowerThresh = numpy.array([0,100,0])
	# Upper Threshold (Bright Green)
        highrThresh = numpy.array([130,255,130])

	# Allowable Range (Detects colours inside threshold)
        img4 = cv2.inRange(cv_image, lowerThresh, highrThresh)
        T = Twist()
        
	# Display target range image
        imshow("Target Range", img4)
        h, w, d = cv_image.shape
        search_top = 3*h/4
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
            T.linear.x = 0.2
            T.angular.z = -float(err) / 100
            self.twist_pub.publish(T)
            
	# --
        #self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        
        # defining image variables
        
        # On init
        #if self.h == 0:
        #    # Create cv_image shape/matrix
        #    self.h, self.w, self.d = cv_image.shape
        #    # DEBUG - Print dimension variables
        #    print self.h, self.w, self.d
        #               
        #    # Set Hz rate for commands
        #    r = rospy.Rate(0.5);
        #    # Move along X dimension for 2 meters (Forward)(Predicted/Not Actual)
        #    T.linear.x = 0.0
        #    # Publish command to robot
        #    self.twist_pub.publish(T)
        #        
        #    # Sleep until command is completed
        #    r.sleep()
        #    # Reset X movement
        #    T.linear.x = 0.0
        #    # Publish empty command
        #    self.twist_pub.publish(T)
        
    # Definition for independant wheel movement
    def forward_kinematics(self, w_l, w_r):
	# Left Wheel
        c_l = self.wheel_radius * w_l
	# Right Wheel
        c_r = self.wheel_radius * w_r
	# Velocity calc
        v = (c_l + c_r) / 2
	# Acceleration calc
        a = (c_l - c_r) / self.robot_radius
        return (v, a)
    
        
# Init ROSPY node
rospy.init_node('image_converter')

# Shortened definition for class
ic = image_converter()

# Block interaction until node is shutdown
rospy.spin()

# Cleanup Windows
destroyAllWindows()

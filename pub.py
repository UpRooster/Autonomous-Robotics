# -*- coding: utf-8 -*-
"""
Created on Thu Feb  9 16:41:42 2017

@author: computing
"""

#!/usr/bin/env python
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv import image_converter

class image_publisher:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/image', Image, latch=True)

    def publish(self, filename):
        cv_image = cv2.imread(filename)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

ic = image_converter()
ip = image_publisher()
rospy.init_node('image_converter', anonymous=True)
ip.publish('blofeld.jpg')

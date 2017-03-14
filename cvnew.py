# Encoding: utf-8
# LTB - 14475946
# CV.py
"""
Created on Tues Mar 14 13:11

@author: Liam T Berridge
"""

import rospy
from classes import classes

class Main:
    def __init__(self):
        # Init vision
        print "Init Vision"
        classes.Vision()

        # Init mapping
        # classes.mapping()

        #Init motion
        # classes.motion()

rospy.init_node('main')
main = Main()
rospy.spin()
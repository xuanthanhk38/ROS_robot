#! /usr/bin/env python

import threading
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import *
import numpy as np
import copy
import time

ranges_filter = []
intensities_filter = []

prefix_time = 3000 #30 seconds
	
rospy.init_node('scan_simulate')  #Initial node for laser filter

scan_pub = rospy.Publisher('scan', LaserScan, queue_size=10) #filter topic publisher

r = rospy.Rate(10)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    filterScan = LaserScan()

    filterScan.header.stamp = current_time
    filterScan.header.frame_id = 'base_scan'
    filterScan.angle_min = 0 # start angle of the scan [rad]
    filterScan.angle_max = math.pi * 2  # end angle of the scan [rad]
    filterScan.angle_increment = 0.0174532923847 # angular distance between measurements [rad]
    filterScan.time_increment = 2.98699997074e-05 # time between measurements [seconds]
    filterScan.range_min = 0.0 # minimum range value [m]
    filterScan.range_max = 3.5 # maximum range value [m]

    filterScan.ranges = []
    filterScan.intensities = []

    scan_pub.publish(filterScan)
    r.sleep()

#!/usr/bin/env python

import gmplot
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32

lat = []
lon = []

gmap3 = gmplot.GoogleMapPlotter(0, 0, 2)

def cb(msg):
	global gmap3
	gmap3.plot(lat, lon, 'cornflowerblue', edge_width = 3.0)
	gmap3.draw('test4.html')

def gps_subscriber_callback(msg):
	global lat, lon
	print(msg.latitude, msg.longitude)
	lat.append(msg.latitude)
	lon.append(msg.longitude)

rospy.init_node('gps_plot')
rospy.Subscriber("gps_front/fix", NavSatFix, gps_subscriber_callback)
rospy.Subscriber("/asdf", Int32, cb)

rospy.spin()



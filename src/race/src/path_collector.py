#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from os.path import expanduser

file_directory = expanduser("~/ISCC_2019/src/race/src/path/path_contest2.txt")


location = []

def odom_callback(msg):
	global location
	location = [msg.pose.pose.position.x, msg.pose.pose.position.y]
	# print(location)

def key_callback(msg):
	global location
	print(file_directory)
	with open(file_directory, 'a') as f:
		f.write(str(location[0]) + ' ' + str(location[1]) + '\n')
		print(location)


rospy.init_node('path_collector')

odom_sub = rospy.Subscriber("odom_front", Odometry, odom_callback)
key_sub = rospy.Subscriber("key", Int8, key_callback)

rospy.spin()

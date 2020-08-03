#!/usr/bin/env python

import rospy
from std_msgs.msg import Time

rospy.init_node("pub2_node")


pub1 = rospy.Publisher('/clock', Time, queue_size=1)


while not rospy.is_shutdown():
	t = rospy.Time.now()
	print(t)
	pub1.publish(t)
#!/usr/bin/env python


import math
from math import sin, cos, pi

import rospy
import tf
from std_msgs.msg import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

rospy.init_node("pub_node")

pub2 = rospy.Publisher('/husky_velocity_controller/odom', Odometry, queue_size=50)

odom_broadcaster = tf.TransformBroadcaster()

x = 0.0
y = 0.0
th = 0.0

vx = 0.1
vy = -0.1
vth = 0.1

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(1.0)

while not rospy.is_shutdown():
	dt = (current_time - last_time).to_sec()
	delta_x = (vx * cos(th) - vy * sin(th)) * dt
	delta_y = (vx * sin(th) + vy * cos(th)) * dt
	delta_th = vth * dt

	x += delta_x
	y += delta_y
	th += delta_th

	odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

	odom_broadcaster.sendTransform(
		(x, y, 0.),
		odom_quat,
		current_time,
		"base_link",
		"odom"
	)

	odom = Odometry()
	odom.header.stamp = current_time
	odom.header.frame_id = "odom"

	odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

	odom.child_frame_id = "base_link"
	odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

	pub2.publish(odom)

	last_time = current_time
	r.sleep()
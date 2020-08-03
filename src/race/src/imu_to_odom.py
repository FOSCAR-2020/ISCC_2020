#!/usr/bin/env python

import rospy
import math

from sensor_msgs.msg import Imu
from race.msg import test
from tf.transformations import euler_from_quaternion

rx = 0.0
ry = 0.0
rz = 0.0

vx = 0.0
vy = 0.0
vz = 0.0

ax = 0.0
ay = 0.0
az = 0.0

pre_vx = 0.0
pre_vy = 0.0
pre_vz = 0.0

pre_ax = 0.0
pre_ay = 0.0
pre_az = 0.0

pre_roll = 0.0
pre_pitch = 0.0
pre_yaw = 0.0

pre_time = False

def antiDerivative(pre, cur, t):
	return (pre + cur)*0.5*t

def processOdom_message(imuMsg):
	global pre_time, rx, ry, rz, vx, vy, vz, ax, ay, az
	global pre_ax, pre_ay, pre_az, pre_vx, pre_vy, pre_vz
	global pre_roll, pre_pitch, pre_yaw

	quaternion = (
		imuMsg.orientation.x,
		imuMsg.orientation.y,
		imuMsg.orientation.z,
		imuMsg.orientation.w)
	(roll,pitch,yaw) = euler_from_quaternion(quaternion)

	ax = imuMsg.linear_acceleration.x
	ay = imuMsg.linear_acceleration.y
	az = imuMsg.linear_acceleration.z

	if(pre_time == False):
		pre_time = imuMsg.header.stamp
		pre_ax = ax
		pre_ay = ay
		pre_az = az
		pre_roll = roll
		pre_pitch = pitch
		pre_yaw = yaw
	# t = imuMsg.header.stamp.secs - pre_time.secs
	t = float(1)/float(34)
	# t = 0.001

	a = (pitch - pre_pitch)/t
	b = (roll - pre_roll)/t
	r = (yaw - pre_yaw)/t

	# ta = 
	# tb = 
	# tr = 


	print(ax)
	print(ay)
	print(az)

	vx = pre_vx + antiDerivative(pre_ax, ax, t)
	vy = pre_vy + antiDerivative(pre_ay, ay, t)
	vz = pre_vz + antiDerivative(pre_az, az, t)

	rx = rx + antiDerivative(pre_vx, vx, t)
	ry = ry + antiDerivative(pre_vy, vy, t)
	rz = rz + antiDerivative(pre_vz, vz, t)

	msg = test()
	msg.x = rx
	msg.y = ry
	msg.z = rz
	imu_pub.publish(msg)

	pre_time = imuMsg.header.stamp

	pre_ax = ax
	pre_ay = ay
	pre_az = az

	pre_vx = vx
	pre_vy = vy
	pre_vz = vz



rospy.init_node("imu_to_odom_node")
imu_sub = rospy.Subscriber('imu', Imu, processOdom_message)
imu_pub = rospy.Publisher('test', test, queue_size=1)
rospy.spin()


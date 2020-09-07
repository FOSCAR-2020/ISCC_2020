#!/usr/bin/env python

import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import TwistWithCovarianceStamped

vel_list = []
time_list = []
time1 = 0

def gps_vel(data):
    global time1
    linear_x = data.twist.twist.linear.x
    linear_y = data.twist.twist.linear.y
    #print("linear x : ", data.twist.twist.linear.x)
    #print("linear y : ", data.twist.twist.linear.y)
    vel = math.sqrt(linear_x**2 + linear_y**2)
    vel_list.append(vel)
    time_list.append(time1)
    time1 += 1
    print(vel)

rospy.init_node('gps_velocity')
rospy.Subscriber('gps_front/fix_velocity', TwistWithCovarianceStamped, gps_vel)

while not rospy.is_shutdown():
    pass

vel_list = np.array(vel_list, dtype=np.float64)
time_list = np.array(time_list, dtype=np.int16)

plt.plot(time_list, vel_list)
plt.show()

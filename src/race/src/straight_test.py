#!/usr/bin/env python

import rospy
from race.msg import drive_values

rospy.init_node("go_straight")
drive_values_pub = rospy.Publisher('control_value', drive_values, queue_size=1)

car_run_speed = 1
max_speed = 5

def auto_drive():
    global car_run_speed
    global max_speed


    if car_run_speed < max_speed:
        car_run_speed += 0.01

    drive_value = drive_values()

    drive_value.throttle = int(car_run_speed)
    drive_value.steering = 0.0

    drive_values_pub.publish(drive_value)

    print("steer : ", drive_value.steering)
    print("throttle : ", drive_value.throttle)


if __name__ == "__main__":
    while not rospy.is_shutdown():
        auto_drive()
        rospy.sleep(0.05)

    

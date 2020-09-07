#!/usr/bin/env python

import rospy
import rospkg
from datetime import datetime
from geometry_msgs.msg import Point

rospack = rospkg.RosPack()
ROS_HOME = rospack.get_path('pure_pursuit')

f = None

def callback(coordinate):
    global f
    f.write(str(coordinate.x) + ' ' + str(coordinate.y) + ' ' + '0' + '\n')


if __name__ == '__main__':
    rospy.init_node("path_maker")

    now = datetime.now()

    f = open(ROS_HOME + "/paths/{}-{}-{}_{}-{}.txt".format(now.year, now.month, now.day, now.hour, now.minute), 'w')
    rospy.Subscriber('utmk_coordinate', Point, callback)


    while not rospy.is_shutdown():
        pass

    f.close()

#!/usr/bin/env python

import rospy
from datetime import datetime
from geometry_msgs.msg import Point

f = None

def callback(coordinate):
    global f
    f.write(str(coordinate.x) + ' ' + str(coordinate.y) + '\n')   


if __name__ == '__main__':
    rospy.init_node("path_maker")

    now = datetime.now()
      
    f = open("paths/{}-{}-{}_{}-{}.txt".format(now.year, now.month, now.day, now.hour, now.minute), 'w')
    rospy.Subscriber('utmk_coordinate', Point, callback)
    
    
    while not rospy.is_shutdown():
        pass
        
    f.close()

#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

f = None

def callback(coordinate):
    global f
    f.write(str(coordinate.x) + ' ' + str(coordinate.y) + '\n')   


if __name__ == '__main__':
    rospy.init_node("path_maker")
    f = open("/home/foscar/ISCC_2019/src/pure_pursuit_test/path.txt", 'w')
    rospy.Subscriber('UTMK_coordinate', Point, callback)
    
    
    while not rospy.is_shutdown():
        pass
        
    f.close()

#!/usr/bin/env python

from pyproj import Proj, transform
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix

pub = None

def wgs84_to_UTMK(data):
    global pub
    latitude = data.latitude
    longitude = data.longitude
    UTMK_coordinate = Point()

    # UTM-K
    proj_UTMK = Proj(init='epsg:5179')

    # WGS84
    proj_WGS84 = Proj(init='epsg:4326')

    x, y = transform(proj_WGS84, proj_UTMK, longitude, latitude)
    UTMK_coordinate.x = x
    UTMK_coordinate.y = y
    UTMK_coordinate.z = 0

    print(UTMK_coordinate.x, UTMK_coordinate.y)

    pub.publish(UTMK_coordinate)


if __name__ == "__main__":
    rospy.init_node("wgs84_to_UTMK")
    rospy.Subscriber("/gps_front/fix", NavSatFix, wgs84_to_UTMK)
    pub = rospy.Publisher("/UTMK_coordinate", Point, queue_size = 1)

    while not rospy.is_shutdown():
        pass

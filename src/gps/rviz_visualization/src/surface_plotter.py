#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import rospkg
import math, sys

rospy.init_node('surface_plotter')
topic = 'visualization_marker_array2'
publisher = rospy.Publisher(topic, MarkerArray, queue_size=100)
rospack = rospkg.RosPack()
markerArray = MarkerArray()
ROS_HOME = rospack.get_path('pure_pursuit')

count = 0
MARKERS_MAX = 100

path_x = []
path_y = []
path_len = 0

# with open(ROS_HOME + "/straight.txt") as f:
with open(ROS_HOME + "/paths/final/visual/surface.txt") as f:
  for line in f.readlines():
    x = float(line.strip().split()[0])
    y = float(line.strip().split()[1])
    path_x.append(x)
    path_y.append(y)
    path_len += 1
    #print(x, y)

rospy.sleep(1)

while path_len > count:
  marker = Marker()
  marker.header.frame_id = "/base_link"
  marker.type = marker.SPHERE
  marker.action = marker.ADD

  marker.scale.x = 0.3
  marker.scale.y = 0.3
  marker.scale.z = 0.3

  marker.color.a = 1.0
  marker.color.r = 1.0
  marker.color.g = 1.0
  marker.color.b = 1.0
  marker.pose.orientation.w = 1.0
  marker.pose.position.x = path_x[count]
  marker.pose.position.y = path_y[count]
  marker.pose.position.z = 0


  markerArray.markers.append(marker)

  id = 0
  for m in markerArray.markers:
    m.id = id
    id += 1

  #print(count)
  publisher.publish(markerArray)

  count += 1
  rospy.sleep(0.0001)

########################
count = 0
MARKERS_MAX = 100

path_x = []
path_y = []
path_len = 0

with open(ROS_HOME + "/paths/final/visual/stop_line.txt") as f:
  for line in f.readlines():
    x = float(line.strip().split()[0])
    y = float(line.strip().split()[1])
    path_x.append(x)
    path_y.append(y)
    path_len += 1
    #print(x, y)

rospy.sleep(1)

while path_len > count:
  marker = Marker()
  marker.header.frame_id = "/base_link"
  marker.type = marker.SPHERE
  marker.action = marker.ADD

  marker.scale.x = 0.5
  marker.scale.y = 0.5
  marker.scale.z = 0.5

  marker.color.a = 1.0
  marker.color.r = 1.0
  marker.color.g = 0
  marker.color.b = 0
  marker.pose.orientation.w = 1.0
  marker.pose.position.x = path_x[count]
  marker.pose.position.y = path_y[count]
  marker.pose.position.z = 0


  markerArray.markers.append(marker)

  id = 0
  for m in markerArray.markers:
    m.id = id
    id += 1

  #print(count)
  publisher.publish(markerArray)

  count += 1
  rospy.sleep(0.0001)

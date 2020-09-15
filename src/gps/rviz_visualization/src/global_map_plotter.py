#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import rospkg
import math, sys

# for test
from geometry_msgs.msg import PointStamped
##


rospy.init_node('global_map_plotter')
topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray, queue_size=100)
rospack = rospkg.RosPack()
ROS_HOME = rospack.get_path('pure_pursuit')
markerArray = MarkerArray()

# for test
target_point_pub = rospy.Publisher("target_point", PointStamped, queue_size=100)
##

count = 0
MARKERS_MAX = 100
mode = 0

path_x = []
path_y = []
modes = []
path_len = 0
current_mode = 0

# with open(ROS_HOME + "/straight.txt") as f:
for path_file in sys.argv[1].split(','):
    with open(ROS_HOME + "/paths/" + path_file + ".txt") as f:
      print("===========>" + ROS_HOME + "/paths/" + path_file+ ".txt")
      for line in f.readlines():
        x = float(line.strip().split()[0])
        y = float(line.strip().split()[1])
        if len(line.strip().split()) >= 3:
          mode = int(line.strip().split()[2])
        path_x.append(x)
        path_y.append(y)
        modes.append(mode)
        path_len += 1
        print(x, y, mode)

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

  if modes[count]%2 == 0:
    marker.color.r = 1
    marker.color.g = 0.5
    marker.color.b = 0


  if modes[count]%2 == 1:
    marker.color.b = 0.0

  marker.pose.orientation.w = 1.0
  marker.pose.position.x = path_x[count]
  marker.pose.position.y = path_y[count]
  marker.pose.position.z = 0

  markerArray.markers.append(marker)

  # Add mode color && tag
  start_x = 0
  start_y = 0
  if modes[count] != current_mode:
    current_mode = modes[count]
    start_x = path_x[count]
    start_y = path_y[count]

    text_marker = Marker()

    text_marker.header.frame_id = "/base_link"
    text_marker.type = marker.TEXT_VIEW_FACING
    text_marker.action = marker.ADD
    text_marker.scale.z = 1

    text_marker.color.a = 1.0
    text_marker.color.r = 0.0
    text_marker.color.g = 1.0
    text_marker.color.b = 0.0

    text_marker.pose.position.x = path_x[count]
    text_marker.pose.position.y = path_y[count]
    text_marker.pose.position.z = 1

    text_marker.text="mode : " + str(modes[count])

    markerArray.markers.append(text_marker)

  id = 0
  for m in markerArray.markers:
    m.id = id
    id += 1

  #print(count)
  publisher.publish(markerArray)

  count += 1

  rospy.sleep(0.001)

# for test
while not rospy.is_shutdown():
  try :
    idx = int(input("input index : "))
    target_point = PointStamped()
    target_point.header.frame_id = "/base_link"
    target_point.point.x = path_x[idx]
    target_point.point.y = path_y[idx]

    target_point_pub.publish(target_point)
  except:
    pass
  rospy.sleep(0.5)
#

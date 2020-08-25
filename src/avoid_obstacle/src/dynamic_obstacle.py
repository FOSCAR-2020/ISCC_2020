#!/usr/bin/env python

import rospy
from obstacle_detector.msg import Obstacles
from race.msg import lane_info, drive_values

drive_values_pub = rospy.Publisher('control_value', drive_values, queue_size=1)

sec = 0

def callback(msg):
      global sec
      center = []

      drive_value = drive_values()
      drive_value.throttle = int(10)
      drive_value.steering = (0)

      """
      if msg.header.stamp.secs == sec:
            pass
      else:
            sec = msg.header.stamp.secs
      """

      for i in msg.circles:
        center.append([i.center.x, i.center.y])

        if i.center.x < 3 and (i.center.y > - 1.5 and i.center.y < 1.5):
          drive_value.throttle = int(0)
          print("Stop!!")

      rospy.loginfo(len(center))
      for i in center:
            print(i)
      print("")

      drive_values_pub.publish(drive_value)

      # rospy.loginfo(len(msg.circles))
      # rospy.loginfo(msg.circles)

def listener():
      rospy.init_node('static_obstacles',anonymous=True)
      rospy.Subscriber("raw_obstacles", Obstacles, callback)
      rospy.spin()

if __name__=='__main__':
      listener()
      
"""
Structure of Obstacles.msg

<msgname>.header
std_msgs/Header header
  uint32 seq 
  time stamp
  string frame_id

<msgname>.segments
obstacle_detector/SegmentObstacle[] segments
  geometry_msgs/Point first_point
    float64 x
    float64 y
    float64 z
  geometry_msgs/Point last_point
    float64 x
    float64 y
    float64 z

<msgname>.circles
obstacle_detector/CircleObstacle[] circles

<msgname>.circles.center.(x / y / z)
  geometry_msgs/Point center
    float64 x
    float64 y
    float64 z
<msgname>.circles.velocity.(x / y /z)
  geometry_msgs/Vector3 velocity
    float64 x
    float64 y
    float64 z
  float64 radius
  float64 true_radius
"""
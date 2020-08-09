#!/usr/bin/env python

import rospy, time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, Pose, PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

class line:
    def __init__(self):
        print("==> init")
        self.current_pose = 0.0
        self.target_point = 0.0

        self.sub_current_pose = rospy.Subscriber("current_pose", PoseStamped, self.currentpose_callback)
        self.sub_target_point = rospy.Subscriber("target_point", PointStamped, self.targetpoint_callback)

    def currentpose_callback(self, msg):
        self.current_pose = msg.pose.position

    def targetpoint_callback(self, msg):
        self.target_point = msg.point


if __name__ == "__main__":
    rospy.init_node('line_pub')
    node = line()

    pub_line = rospy.Publisher('line', Marker, queue_size=1)

    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "/base_link"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        # marker scale
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        # marker color
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        if type(node.current_pose) is float or type(node.target_point) is float:
            time.sleep(0.1)
            
        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = node.current_pose.x
        marker.pose.position.y = node.current_pose.y
        marker.pose.position.z = 1.0

        # marker line points
        marker.points = []
        # first point
        first_line_point = Point()
        

        first_line_point.x = 0
        first_line_point.y = 0
        first_line_point.z = 0.0
        marker.points.append(first_line_point)

        # second point
        second_line_point = Point()
        second_line_point.x = node.target_point.x - node.current_pose.x
        second_line_point.y = node.target_point.y - node.current_pose.y
        second_line_point.z = 0.0

        marker.points.append(second_line_point)
        print("======================================>")
        print(node.current_pose.x, node.target_point.x)

        # Publish the Marker
        pub_line.publish(marker)

        rospy.sleep(0.5)

    
#!usr/bin/env python
# temporary file

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

test_msg = PoseStamped()
pub = None

def callback(msg):
    print("hello")
    global pub
    test_msg = msg
    test_msg.header.frame_id = "/base_link"

    pub.publish(test_msg)

rospy.init_node('listener', anonymous=True)
pub = rospy.Publisher("current_pose2", PoseStamped, queue_size=1)
rospy.Subscriber("current_pose", PoseStamped, callback)

if __name__ == "__main__":
    while not rospy.is_shutdown():
        pass

    print("what happend")

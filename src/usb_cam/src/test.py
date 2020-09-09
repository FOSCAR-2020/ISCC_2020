import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

rospy.init_node("video_test")
pub = rospy.Publisher('usb_cam/image_raw', Image, queue_size=10)

cap = cv2.VideoCapture('longtest.mov')
bridge = CvBridge()

while(not rospy.is_shutdown() and cap.isOpened()):
    ret, frame = cap.read()
    #cv2.imshow('frame',frame)
    msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
    pub.publish(msg)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()

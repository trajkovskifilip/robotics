#!/usr/bin/env python
import rospy
import time
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# initialize the camera and grab a reference to the raw camera capture
bridge = CvBridge()
capture = cv2.VideoCapture(0) # Laptop's camera

def camera_node():
    global bridge
    global capture

    rospy.init_node('camera_node')
    pub = rospy.Publisher('image', Image, queue_size=5)
    rate = rospy.Rate(20)

    # allow the camera to warmup
    time.sleep(0.1)

    while not rospy.is_shutdown():
        # capture frames from the camera
        ret, frame = capture.read()

        if ret:
            img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            pub.publish(img_msg)

        rate.sleep()

if __name__ == "__main__":
    camera_node()

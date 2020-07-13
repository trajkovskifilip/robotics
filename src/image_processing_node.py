#!/usr/bin/env python
import rospy
import cv2
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from robotics.msg import Location
import cv2.cv as cv
import numpy as np
import imutils

bridge = CvBridge()

def callback(img_msg):
    global bridge
    pub = rospy.Publisher("location", Location)
    whiteLower = (158, 136, 155)
    whiteUpper = (255, 255, 255)

    # grab the current image
    image = None
    try:
        image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(image, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "white", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, whiteLower, whiteUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        msg = Location(x, y)
        pub.publish(msg)


def image_processing_node():
    rospy.init_node('image_processing_node')
    rospy.Subscriber("image", Image, callback)
    rospy.spin()

if __name__ == "__main__":
    image_processing_node()

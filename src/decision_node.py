#!/usr/bin/env python

import rospy
import serial
import time
import numpy as np
import string
from lib_robotis import *
from robotics.msg import Location

ser = serial.Serial('COM4', 9600)

dyn = USB2Dynamixel_Device('COM3')
# find_servos(dyn)

s1 = Robotis_Servo(dyn, 7)  # bottom
s2 = Robotis_Servo(dyn, 6)  # first joint
s3 = Robotis_Servo(dyn, 5)  # second joint
s4 = Robotis_Servo(dyn, 4)  # third joint
s5 = Robotis_Servo(dyn, 3)  # rotation
s6 = Robotis_Servo(dyn, 2)  # grasp

RADIANS = 0
DEGREES = 1

l1 = 17.5  # Length of link 1
l2 = 19.5  # length of link 2

def callback(data):
    x = data[0]
    y = data[1]
    
    goTo(invKin(x,y))
    time.sleep(1)
    goToTrash()
    time.sleep(1)
    goToRest()

def invKin(x, y, angleMode=DEGREES):
    """Returns the angles of the first two links
    in the robotic arm as a list.
    returns -> (th1, th2)
    input:
    x - The x coordinate of the effector
    y - The y coordinate of the effector
    angleMode - tells the function to give the angle in
                degrees/radians. Default is degrees
    output:
    th1 - angle of the first link w.r.t ground
    th2 - angle of the second link w.r.t the first"""

    # stuff for calculating th2
    r_2 = x ** 2 + y ** 2
    l_sq = l1 ** 2 + l2 ** 2
    term2 = (r_2 - l_sq) / (2 * l1 * l2)
    term1 = ((1 - term2 ** 2) ** 0.5) * -1
    # calculate th2
    th2 = math.atan2(term1, term2)
    th2 = -1 * th2

    # Stuff for calculating th2
    k1 = l1 + l2 * math.cos(th2)
    k2 = l2 * math.sin(th2)
    r = (k1 ** 2 + k2 ** 2) ** 0.5
    gamma = math.atan2(k2, k1)
    # calculate th1
    th1 = math.atan2(y, x) - gamma

    if (angleMode == RADIANS):
        return th1, th2
    else:
        return math.degrees(th1), math.degrees(th2)


print np.absolute(invKin(36, -5))


def goTo(arrayXY):
    theta1 = np.absolute(arrayXY[0])
    theta2 = np.absolute(arrayXY[1])

    s3.move_angle(math.radians(np.absolute(theta1 - theta2) + 12))
    time.sleep(0.3)
    s5.move_angle(math.radians(theta2), blocking=False)
    time.sleep(0.3)
    s2.move_angle(math.radians(theta1), blocking=False)
    s6.move_angle(math.radians(-theta1), blocking=True)


def goToRest():
    theta1 = 95
    theta2 = 0
    s4.move_angle(math.radians(90), blocking=False)
    s5.move_angle(math.radians(theta2), blocking=False)
    time.sleep(0.5)
    s2.move_angle(math.radians(theta1), blocking=False)
    s6.move_angle(math.radians(-theta1), blocking=True)
    time.sleep(0.5)
    s3.move_angle(math.radians(0))


def goToTrash():
    theta1 = 60
    theta2 = 45
    s1.move_angle(math.radians(-90), blocking=False)
    time.sleep(0.5)
    s4.move_angle(math.radians(45), blocking=False)
    s5.move_angle(math.radians(theta2), blocking=False)
    time.sleep(0.5)
    s2.move_angle(math.radians(theta1), blocking=False)
    s6.move_angle(math.radians(-theta1), blocking=True)
    s3.move_angle(math.radians(np.absolute(theta1 - theta2)))
    time.sleep(0.5)
    s4.move_angle(math.radians(45), blocking=False)


def decision_node():
    rospy.init_node('decision_node')
    rospy.Subscriber("location", Location, callback)
    rospy.spin()

if __name__ == "__main__":
    decision_node()

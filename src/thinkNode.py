#!/usr/bin/env python

import rospy
import math
import time
from pyswip import Prolog
from geometry_msgs.msg import Twist
from kobukiROSindigo.msg import Status

publisher_status = rospy.Publisher('kobuki_velocity', Twist, queue_size=1)
kobukiStatus = Status() # The updated status of the robot

def decisionCallback(kobukiStatus):
    west = kobukiStatus.bumperW
    north = kobukiStatus.bumperN
    est = kobukiStatus.bumperE
    rospy.loginfo('west: {}, north: {}, est: {}'.format(west, north, est))

def think():
    rospy.init_node('act')
    rospy.Subscriber("/kobuki_status", Status, decisionCallback)
    rospy.spin()

if __name__ == '__main__':
    try:
        think()
    except rospy.ROSInterruptException:
        pass

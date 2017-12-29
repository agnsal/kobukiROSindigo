#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def odomCallback(data):
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    rospy.loginfo('x: {}, y: {}'.format(x, y))

def main():
    rospy.init_node('sense')
    rospy.Subscriber("/odom", Odometry, odomCallback)
    rospy.spin()
  
if __name__ == '__main__':
    main()

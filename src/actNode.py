#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from kobukiROSindigo.msg import Status

action = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=1)

def actionCallback(movement):
    print('action: ', movement)
    action.publish(movement)

def act():
    rospy.init_node('act')
    rospy.Subscriber("/kobuki_velocity", Twist, actionCallback)
    rospy.spin()

if __name__ == '__main__':
    try:
        act()
    except rospy.ROSInterruptException:
        pass

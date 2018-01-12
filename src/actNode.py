#!/usr/bin/env python

'''
Copyright 2017-2018 Agnese Salutari.
Licensed under the Apache License, Version 2.0 (the "License"); 
you may not use this file except in compliance with the License. 
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on 
an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
See the License for the specific language governing permissions and limitations under the License
'''

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

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
import time
import cv2, cv_bridge
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import PowerSystemEvent
from kobukiROSindigo.msg import Status
from sensor_msgs.msg import Image

pubKobukiStatus = rospy.Publisher('kobuki_status', Status, queue_size=1)
kobukiStatus = Status()  # The updated status of the robot
bridge = cv_bridge.CvBridge()
cv2.namedWindow("robotView", 1)

def cameraCallback(data):
    sUnixTimestamp = int(time.time())  # Timestamp in seconds
    if sUnixTimestamp - kobukiStatus.lastTime > kobukiStatus.deltaTime:
        image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        cv2.imshow("robotView", image)
        cv2.waitKey(3)  # Display a frame for 3 ms
        rospy.loginfo('cameraTimestamp: {}'.format(sUnixTimestamp))
        kobukiStatus.lastTime = int(time.time())

def odometryCallback(data):
    sUnixTimestamp = int(time.time())  # Timestamp in seconds
    kobukiStatus.odometryX = data.pose.pose.position.x
    kobukiStatus.odometryY = data.pose.pose.position.y
    kobukiStatus.odometryZ = data.pose.pose.position.z
    if sUnixTimestamp - kobukiStatus.lastTime > kobukiStatus.deltaTime:
        print 'Delta time ok: ' + str(sUnixTimestamp - kobukiStatus.lastTime)
        pubKobukiStatus.publish(kobukiStatus)
        rospy.loginfo('x: {}, y: {}, z: {}'.format(kobukiStatus.odometryX, kobukiStatus.odometryY, kobukiStatus.odometryZ))
        kobukiStatus.lastTime = int(time.time())
    
def bumperCallback(data):
    sUnixTimestamp = int(time.time())  # Timestamp in seconds
    state = data.state
    bumper = data.bumper
    kobukiStatus.bumperE = False
    kobukiStatus.bumperW = False
    kobukiStatus.bumperN = False
    if bumper == 0:
        kobukiStatus.bumperW = state
    elif bumper == 1:
        kobukiStatus.bumperN = state
    else:   
        kobukiStatus.bumperE = state
    if sUnixTimestamp - kobukiStatus.lastTime > kobukiStatus.deltaTime:
        print 'Delta time ok: ' + str(sUnixTimestamp - kobukiStatus.lastTime)
        pubKobukiStatus.publish(kobukiStatus)
        rospy.loginfo('bumperW: {}, bumberN: {}, bumperE: {}'.format(kobukiStatus.bumperW, kobukiStatus.bumperN, kobukiStatus.bumperE))
        kobukiStatus.lastTime = int(time.time())
        
def powerCallback(data):
    sUnixTimestamp = int(time.time())  # Timestamp in seconds
    if   ( data.event == PowerSystemEvent.UNPLUGGED ) :
        kobukiStatus.power = PowerSystemEvent.UNPLUGGED
        rospy.loginfo("Robot unplugged")
    elif ( data.event == PowerSystemEvent.PLUGGED_TO_ADAPTER ) :
        kobukiStatus.power = PowerSystemEvent.PLUGGED_TO_ADAPTER
        rospy.loginfo("Robot plugged to adapter")
    elif ( data.event == PowerSystemEvent.PLUGGED_TO_DOCKBASE ) :
        kobukiStatus.power = PowerSystemEvent.PLUGGED_TO_DOCKBASE
        rospy.loginfo("Robot plugged to docking base")
    elif ( data.event == PowerSystemEvent.CHARGE_COMPLETED ) :
        kobukiStatus.power = PowerSystemEvent.CHARGE_COMPLETED
        rospy.loginfo("Robot charge completed")
    elif ( data.event == PowerSystemEvent.BATTERY_LOW ) :
        kobukiStatus.power = PowerSystemEvent.BATTERY_LOW
        rospy.loginfo("Robot battery low")
    elif ( data.event == PowerSystemEvent.BATTERY_CRITICAL ) :
        kobukiStatus.power = PowerSystemEvent.BATTERY_CRITICAL
        rospy.loginfo("Robot battery critical")
    else:
        rospy.loginfo("WARN: Unexpected power system event: %d"%(data.event))
    if sUnixTimestamp - kobukiStatus.lastTime > kobukiStatus.deltaTime:
        print 'Delta time ok: ' + str(sUnixTimestamp - kobukiStatus.lastTime)
        pubKobukiStatus.publish(kobukiStatus)
        rospy.loginfo('power: {}'.format(kobukiStatus.power))
        kobukiStatus.lastTime = int(time.time())
    
def sense():
    rospy.init_node('sense')
    rospy.Subscriber("/odom", Odometry, odometryCallback)
    rospy.Subscriber("/mobile_base/events/power_system", PowerSystemEvent, powerCallback)
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bumperCallback)
    rospy.Subscriber("camera/rgb/image_raw", Image, cameraCallback)
    rospy.spin()
  
if __name__ == '__main__':
    try:
        kobukiStatus.lastTime = 0
        kobukiStatus.deltaTime = 0.5
        sense()
    except rospy.ROSInterruptException:
        pass

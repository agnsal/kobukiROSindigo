#!/usr/bin/env python

import rospy
import cv2, cv_bridge
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import PowerSystemEvent
from kobukiROSindigo.msg import Status
from sensor_msgs.msg import Image

pubKobukiStatus = rospy.Publisher('kobuki_status', Status, queue_size=1)
kobukiStatus = Status()  # The updated status of the robot
bridge = cv_bridge.CvBridge()
cv2.namedWindow("robotWiew", 1)

def cameraCallback(data):
    sUnixTimestamp = int(time.time())  # Timestamp in seconds
	image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
	cv2.imshow("robotWiew", image)
	cv2.waitKey(3)  # Display a frame for 3 ms
    rospy.loginfo('cameraTimestamp: {}'.format(sUnixTimestamp))

def odometryCallback(data):
    kobukiStatus.odometryX = data.pose.pose.position.x
    kobukiStatus.odometryY = data.pose.pose.position.y
    kobukiStatus.odometryZ = data.pose.pose.position.z
    pubKobukiStatus.publish(kobukiStatus)
    rospy.loginfo('x: {}, y: {}, z: {}'.format(kobukiStatus.odometryX, kobukiStatus.odometryY, kobukiStatus.odometryZ))
    
def bumperCallback(data):
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
    pubKobukiStatus.publish(kobukiStatus)
    rospy.loginfo('bumperW: {}, bumberN: {}, bumperE: {}'.format(kobukiStatus.bumperW, kobukiStatus.bumperN, kobukiStatus.bumperE))

def powerCallback(data):
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
    pubKobukiStatus.publish(kobukiStatus)
    rospy.loginfo('power: {}'.format(kobukiStatus.power))
    
def sense():
    rospy.init_node('sense')
    rospy.Subscriber("/odom", Odometry, odometryCallback)
    rospy.Subscriber("/mobile_base/events/power_system", PowerSystemEvent, powerCallback)
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bumperCallback)
    rospy.Subscriber("camera/rgb/image_raw", Image, cameraCallback)
    rospy.spin()
  
if __name__ == '__main__':
    try:
        sense()
    except rospy.ROSInterruptException:
        pass

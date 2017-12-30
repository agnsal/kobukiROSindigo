#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import PowerSystemEvent
from kobuki_project.msg import Status

publisher_status = rospy.Publisher('kobuki_status', Status, queue_size=1)
kobukiStatus = Status()  # The updated status of the robot

def odometryCallback(data):
    kobukiStatus.odometryX = data.pose.pose.position.x
    kobukiStatus.odometryY = data.pose.pose.position.y
    kobukiStatus.odometryZ = data.pose.pose.position.z
    rospy.loginfo('x: {}, y: {}, z: {}'.format(kobukiStatus.odometryX, kobukiStatus.odometryY, kobukiStatus.odometryZ))
    
def bump(bumper):
    state = bumper.state
    bumper = bumper.bumper
    if bumper == 0:
        kobukiStatus.bumperW=state
    elif bumper == 1:
        kobukiStatus.bumperN=state
    else:   
        kobukiStatus.bumperE=state
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
    rospy.loginfo('power: {}'.format(kobukiStatus.power))

def main():
    rospy.init_node('sense')
    rospy.Subscriber("/odom", Odometry, odometryCallback)
    rospy.Subscriber("/mobile_base/events/power_system", PowerSystemEvent, powerCallback)
    rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bumperCallback)
    rospy.spin()
  
if __name__ == '__main__':
    main()

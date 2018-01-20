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
import math
import random
from geometry_msgs.msg import Twist
from kobukiROSindigo.msg import Status
import logging

logging.basicConfig(level=logging.INFO)

pubKobukiVelocity = rospy.Publisher('kobuki_velocity', Twist, queue_size=1)
kobukiStatus = Status()  # The updated status of the robot
kobukiDecisionVelocity = Twist()

def decisionCallbackRandomPy(kobukiStatus):
    print 'Decision taking started...'
    luck = random.randint(0,3)
    if luck == 1:  #  Casual move
        luckyPi = random.randint(1,4)
        kobukiDecisionVelocity.linear.x = -0.15  # Go back at 0.15 m/s
        kobukiDecisionVelocity.linear.y = 0.0
        kobukiDecisionVelocity.angular.z = luckyPi*(-1.5)
    else:
        west = kobukiStatus.bumperW
        north = kobukiStatus.bumperN
        east = kobukiStatus.bumperE
        rospy.loginfo('west: {}, north: {}, est: {}'.format(west, north, east))
        if north is False and east is False and west is False:
            kobukiDecisionVelocity.linear.x = 0.15  # Go forward at 0.15 m/s
            kobukiDecisionVelocity.linear.y = 0.0
            kobukiDecisionVelocity.angular.z = 0.0
            pubKobukiVelocity.publish(kobukiDecisionVelocity)
        elif east is False:
            kobukiDecisionVelocity.linear.x = -0.15
            kobukiDecisionVelocity.linear.y = 0.0
            kobukiDecisionVelocity.angular.z = -1.0  # Turn Right at 1 rad/s
        elif west is False:
            kobukiDecisionVelocity.linear.x = -0.15
            kobukiDecisionVelocity.linear.y = 0.0
            kobukiDecisionVelocity.angular.z = 1.0  # Turn Left at 1rad/s
        elif (north and east and west):
            kobukiDecisionVelocity.linear.x = -0.15 
            kobukiDecisionVelocity.linear.y = 0.0
            kobukiDecisionVelocity.angular.z = 4.0  # Turn Left at 4 rad/s
        else:  # stay
            kobukiDecisionVelocity.linear.x = 0.0
            kobukiDecisionVelocity.linear.y = 0.0  
            kobukiDecisionVelocity.angular.z = 0.0
    pubKobukiVelocity.publish(kobukiDecisionVelocity)
    rospy.loginfo('decisionVelocity.x: {}, decisionVelocity.y: {}, decisionVelocity.z: {}'.format(kobukiDecisionVelocity.linear.x , kobukiDecisionVelocity.linear.y, kobukiDecisionVelocity.angular.z))

def decisionCallbackPy(kobukiStatus):
    print 'Decision taking started...'
    west = kobukiStatus.bumperW
    north = kobukiStatus.bumperN
    east = kobukiStatus.bumperE
    rospy.loginfo('west: {}, north: {}, est: {}'.format(west, north, east))
    if north is False and east is False and west is False:
        kobukiDecisionVelocity.linear.x = 0.15  # Go forward at 0.15 m/s
        kobukiDecisionVelocity.linear.y = 0.0
        kobukiDecisionVelocity.angular.z = 0.0
        pubKobukiVelocity.publish(kobukiDecisionVelocity)
    elif east is False:
        kobukiDecisionVelocity.linear.x = -0.15
        kobukiDecisionVelocity.linear.y = 0.0
        kobukiDecisionVelocity.angular.z = -1.0  # Turn Right at 1 rad/s
    elif west is False:
        kobukiDecisionVelocity.linear.x = -0.15
        kobukiDecisionVelocity.linear.y = 0.0
        kobukiDecisionVelocity.angular.z = 1.0  # Turn Left at 1rad/s
    elif (north and east and west):
        kobukiDecisionVelocity.linear.x = -0.15 
        kobukiDecisionVelocity.linear.y = 0.0
        kobukiDecisionVelocity.angular.z = 4.0  # Turn Left at 4 rad/s
    else:  # stay
        kobukiDecisionVelocity.linear.x = 0.0
        kobukiDecisionVelocity.linear.y = 0.0  
        kobukiDecisionVelocity.angular.z = 0.0
    pubKobukiVelocity.publish(kobukiDecisionVelocity)
    rospy.loginfo('decisionVelocity.x: {}, decisionVelocity.y: {}, decisionVelocity.z: {}'.format(kobukiDecisionVelocity.linear.x , kobukiDecisionVelocity.linear.y, kobukiDecisionVelocity.angular.z))    

def waitForDevilResponse():
    print 'Waiting for response...'
    patience = True
    while(patience):
        try:
            open('response.txt')
            patience = False
        except:
            print 'Waiting for Devil response...'
            rospy.sleep(0.1)

def waitForDevilReady():
    print 'Waiting for attention...'
    patience = True
    while(patience):
        try:
            open('query.txt')
            trospy.sleep(0.1)
        except:
            patience = False
            print 'Devil is available'
    
# The following callback uses Datalog or Prolog Daemon Python3 program.
# If you don't want to use Datalog or Prolog, use decisionCallbackRandomPy or decisionCallbackPy instead.
def logCallbackDatalog(kobukiStatus):
    print 'Decision taking started...'
    west = kobukiStatus.bumperW
    north = kobukiStatus.bumperN
    east = kobukiStatus.bumperE
    perceptionBumper = [['bumperW', west], ['bumperN', north], ['bumperE', east]]
    print 'Decision taking started...'
    bumperFact = str(perceptionBumper).replace('"', "'")
    print 'Fact: ' + bumperFact
    waitForDevilReady()
    # Put data on query file
    outFile = open("query.txt", "w")
    outFile.write(bumperFact + '\n')
    outFile.close()
    waitForDevilResponse()
    print 'Devil wrote back'
    response = open('response.txt')
    toDo = response.readline()
    response.close()
    print 'Davil said: ' + toDo
    if 'GoStraight' in toDo:
        kobukiDecisionVelocity.linear.x = 0.15 # Go forward at 0.15 m/s
        kobukiDecisionVelocity.linear.y = 0.0
        kobukiDecisionVelocity.angular.z = 0.0
    elif 'TurnEast' in toDo:
        kobukiDecisionVelocity.linear.x = -0.15  # Go back at 0.1 m/s
        kobukiDecisionVelocity.linear.y = 0.0
        kobukiDecisionVelocity.angular.z = -1.0  # Turn Right at 1 rad/s
    elif 'TurnWest' in toDo:
        kobukiDecisionVelocity.linear.x = -0.15  # Go back at 0.1 m/s
        kobukiDecisionVelocity.linear.y = 0.0
        kobukiDecisionVelocity.angular.z = 1  # Turn Left at 1 rad/s
    elif 'TurnSouth' in toDo:
        kobukiDecisionVelocity.linear.x = -0.15  # Go back at 0.1 m/s
        kobukiDecisionVelocity.linear.y = 0.0
        kobukiDecisionVelocity.angular.z = 4.0  # Turn Left at 4 rad/s
    else:  # toDo == 'Stay'
        kobukiDecisionVelocity.linear.x = 0.0
        kobukiDecisionVelocity.linear.y = 0.0  
        kobukiDecisionVelocity.angular.z = 0.0
    pubKobukiVelocity.publish(kobukiDecisionVelocity)
    rospy.loginfo('decisionVelocity.x: {}, decisionVelocity.y: {}, decisionVelocity.z: {}'.format(kobukiDecisionVelocity.linear.x , kobukiDecisionVelocity.linear.y, kobukiDecisionVelocity.angular.z))
    print 'Thinking completed'
    os.remove('response.txt')
    print 'Previous knowledge retracted...'
    
def think():
    rospy.init_node('think')
    rospy.Subscriber("/kobuki_status", Status, logCallbackDatalog)
    rospy.spin()

if __name__ == '__main__':
    try:
        think()
    except rospy.ROSInterruptException:
        pass

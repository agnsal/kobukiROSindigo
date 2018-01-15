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
import time
import random
from pyswip import Prolog
from pyDatalog import pyEngine
from pyDatalog import pyDatalog
from geometry_msgs.msg import Twist
from kobukiROSindigo.msg import Status
from threading import Thread
import logging

logging.basicConfig(level=logging.INFO)

class thinkingThread(Thread):
    def __init__(self, callback):
        Thread.__init__(self)
        self.callback = callback

    def run(self):
        data = "hello"
        self.callback(data)


class Manager():
    def Test(self):
        MyThread(self.on_thread_finished).start()

    def on_thread_finished(self, data):
        print "on_thread_finished:", data


pubKobukiVelocity = rospy.Publisher('kobuki_velocity', Twist, queue_size=1)
kobukiStatus = Status()  # The updated status of the robot
prologEngine = Prolog()
kobukiDecisionVelocity = Twist()
bumperEventList = []

def learn(prologFilePath):  # Function not used because Pyswip is not compatible
    '''
    Learns from a SWI Prolog file.
    :param prologFilePath: The path of the Prolog (.pl or .txt) file we need to use.
    :return:
    '''
    print 'Learning started...'
    assert isinstance(prologFilePath, str)
    prologEngine.consult(prologFilePath)
    print 'Learning finished'

# The following function gives the "Segmentation fault (core dumped)" error because of Pyswip
def decisionCallback(kobukiStatus):
    print 'Decision taking started...'
    west = kobukiStatus.bumperW
    north = kobukiStatus.bumperN
    east = kobukiStatus.bumperE
    perceptionBumper = [['bumperW', west], ['bumperN', north], ['bumperE', east]]
    print(perceptionBumper)
    prologEngine.assertz('perceptionBumper(' + str(perceptionBumper).replace('"', "'") + ')')
    print 'New knowledge taken...'
    rospy.loginfo('west: {}, north: {}, est: {}'.format(west, north, east))
    try:
        out = list(prologEngine.query('takeDecision(D)'))
    except:  # If Prolog doesn't work (maybe because it can't receive data from sensors) the robot has to stay
        out = []
    # print(out)  # test
    if len(out) > 0 :  # If we can take more than one decision, we take the 1st one
        # print(out[0]['D'])  # Test
        toDo = out[0]['D']
    else:
        toDo = 'Stay'
    if toDo == 'GoStraight':
        kobukiDecisionVelocity.linear.x = 0.15 # Go forward at 0.15 m/s
        kobukiDecisionVelocity.linear.y = 0.0
        kobukiDecisionVelocity.angular.z = 0.0
    elif toDo == 'TurnEast':
        kobukiDecisionVelocity.linear.x = -0.1  # Go back at 0.1 m/s
        kobukiDecisionVelocity.linear.y = 0.0
        kobukiDecisionVelocity.angular.z = -1.0  # Turn Right at 1 rad/s
    elif toDo == 'TurnWest':
        kobukiDecisionVelocity.linear.x = -0.1  # Go back at 0.1 m/s
        kobukiDecisionVelocity.linear.y = 0.0
        kobukiDecisionVelocity.angular.z = 1  # Turn Left at 1 rad/s
    elif toDo == 'TurnSouth':
        kobukiDecisionVelocity.linear.x = -0.1  # Go back at 0.1 m/s
        kobukiDecisionVelocity.linear.y = 0.0
        kobukiDecisionVelocity.angular.z = 4.0  # Turn Left at 4 rad/s
    else:  # toDo == 'Stay'
        kobukiDecisionVelocity.linear.x = 0.0
        kobukiDecisionVelocity.linear.y = 0.0  
        kobukiDecisionVelocity.angular.z = 0.0
    pubKobukiVelocity.publish(kobukiDecisionVelocity)
    rospy.loginfo('decisionVelocity.x: {}, decisionVelocity.y: {}, decisionVelocity.z: {}'.format(kobukiDecisionVelocity.linear.x , kobukiDecisionVelocity.linear.y, kobukiDecisionVelocity.angular.z))
    prologEngine.retractall('perceptionBumper(_)')
    print 'Previous knowledge retracted...'

def decisionCallbackRandomPy(kobukiStatus):
    print 'Decision taking started...'
    luck = random.randint(0,3)
    if luck == 1:  #  Casual move
        luckyPi = random.randint(1,4)
        kobukiDecisionVelocity.linear.x = -0.1
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
            kobukiDecisionVelocity.linear.x = -0.1
            kobukiDecisionVelocity.linear.y = 0.0
            kobukiDecisionVelocity.angular.z = -1.0  # Turn Right at 1 rad/s
        elif west is False:
            kobukiDecisionVelocity.linear.x = -0.1
            kobukiDecisionVelocity.linear.y = 0.0
            kobukiDecisionVelocity.angular.z = 1.0  # Turn Left at 1rad/s
        elif (north and east and west):
            kobukiDecisionVelocity.linear.x = -0.1 
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
        kobukiDecisionVelocity.linear.x = -0.1
        kobukiDecisionVelocity.linear.y = 0.0
        kobukiDecisionVelocity.angular.z = -1.0  # Turn Right at 1 rad/s
    elif west is False:
        kobukiDecisionVelocity.linear.x = -0.1
        kobukiDecisionVelocity.linear.y = 0.0
        kobukiDecisionVelocity.angular.z = 1.0  # Turn Left at 1rad/s
    elif (north and east and west):
        kobukiDecisionVelocity.linear.x = -0.1 
        kobukiDecisionVelocity.linear.y = 0.0
        kobukiDecisionVelocity.angular.z = 4.0  # Turn Left at 4 rad/s
    else:  # stay
        kobukiDecisionVelocity.linear.x = 0.0
        kobukiDecisionVelocity.linear.y = 0.0  
        kobukiDecisionVelocity.angular.z = 0.0
    pubKobukiVelocity.publish(kobukiDecisionVelocity)
    rospy.loginfo('decisionVelocity.x: {}, decisionVelocity.y: {}, decisionVelocity.z: {}'.format(kobukiDecisionVelocity.linear.x , kobukiDecisionVelocity.linear.y, kobukiDecisionVelocity.angular.z))    

# The following function uses Datalog 
def datalogLearning(self, filePath = 'behaviour.dl'):
    pyDatalog.create_terms('D')
    pyDatalog.create_terms('N')
    pyDatalog.create_terms('W')
    pyDatalog.create_terms('E')
    rules = open(filePath).read()
    pyDatalog.load(rules)
    
# The following function uses Datalog 
def speecunialityCallbackDatalog(kobukiStatus):
    print 'Decision taking started...'
    west = kobukiStatus.bumperW
    north = kobukiStatus.bumperN
    east = kobukiStatus.bumperE
    perceptionBumper = [['bumperW', west], ['bumperN', north], ['bumperE', east]]
    bumperEventList.append(perceptionBumper)
    rate = rospy.Rate(1) # 1 Hz
    # Do stuff, maybe in a while loop
    print 'Decision taking started...'
    print(perceptionBumper)
    fact = str(perceptionBumper).replace('"', "'")
    print 'Fact: ' + fact
    pyDatalog.assert_fact('perceptionBumper', fact) #  Asserts a new fact
    print 'New knowledge taken...'
    rospy.loginfo('west: {}, north: {}, est: {}'.format(west, north, east))
    try:
        out = pyDatalog.ask('takeDecision(D)')
    except:  # If Prolog doesn't work (maybe because it can't receive data from sensors) the robot has to stay
        out = []
    # print(out)  # test
    if len(out) > 0 :  # If we can take more than one decision, we take the 1st one
        # print(out[0]['D'])  # Test
        toDo = out[0]['D']
    else:
        toDo = 'Stay'
    if toDo == 'GoStraight':
        kobukiDecisionVelocity.linear.x = 0.15 # Go forward at 0.15 m/s
        kobukiDecisionVelocity.linear.y = 0.0
        kobukiDecisionVelocity.angular.z = 0.0
    elif toDo == 'TurnEast':
        kobukiDecisionVelocity.linear.x = -0.1  # Go back at 0.1 m/s
        kobukiDecisionVelocity.linear.y = 0.0
        kobukiDecisionVelocity.angular.z = -1.0  # Turn Right at 1 rad/s
    elif toDo == 'TurnWest':
        kobukiDecisionVelocity.linear.x = -0.1  # Go back at 0.1 m/s
        kobukiDecisionVelocity.linear.y = 0.0
        kobukiDecisionVelocity.angular.z = 1  # Turn Left at 1 rad/s
    elif toDo == 'TurnSouth':
        kobukiDecisionVelocity.linear.x = -0.1  # Go back at 0.1 m/s
        kobukiDecisionVelocity.linear.y = 0.0
        kobukiDecisionVelocity.angular.z = 4.0  # Turn Left at 4 rad/s
    else:  # toDo == 'Stay'
        kobukiDecisionVelocity.linear.x = 0.0
        kobukiDecisionVelocity.linear.y = 0.0  
        kobukiDecisionVelocity.angular.z = 0.0
    pubKobukiVelocity.publish(kobukiDecisionVelocity)
    rospy.loginfo('decisionVelocity.x: {}, decisionVelocity.y: {}, decisionVelocity.z: {}'.format(kobukiDecisionVelocity.linear.x , kobukiDecisionVelocity.linear.y, kobukiDecisionVelocity.angular.z))
    pyDatalog.retract_fact('perceptionBumper', fact)
    print 'Previous knowledge retracted...'
    rate.sleep() # Sleeps for 1/rate sec
    print 'Thinking completed'
    
def think():
    # learn('behaviour.pl')  # Pyswip is not compatible
    rospy.init_node('think')
    datalogLearning('root/catkin_ws/src/kobukiROSindigo/src/behaviour.dl')
    rospy.Subscriber("/kobuki_status", Status, speecunialityCallbackDatalog)
    rospy.spin()

if __name__ == '__main__':
    try:
        think()
    except rospy.ROSInterruptException:
        pass

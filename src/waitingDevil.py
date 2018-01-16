#!/usr/bin/env python3

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

import os
import time
from pyDatalog import pyEngine
from pyDatalog import pyDatalog
import logging
import ast


logging.basicConfig(level=logging.INFO)

behaviourFilePath = 'behaviour.dl'
pyDatalog.create_terms('hasValue, bumperW, W, N, E, perceptionBumper, takeDecision, hasValue, X, Y, D')

print("Learning started...")

hasValue('bumperW', W) <= perceptionBumper([['bumperW', W], X, Y])
hasValue('bumperN', N) <= perceptionBumper([X, ['bumperN', N], Y])
hasValue('bumperE', E) <= perceptionBumper([X, Y, ['bumperE', E]])

takeDecision('TurnSouth') <= hasValue('bumperN', 'True') & hasValue('bumperW', 'True') & hasValue('bumperE', 'True')
takeDecision('GoStraight') <= hasValue('bumperN', 'False') & hasValue('bumperW', 'False') & hasValue('bumperE', 'False')
takeDecision('TurnWest') <= hasValue('bumperN', 'True') & hasValue('bumperW', 'False')
takeDecision('TurnEast') <= hasValue('bumperN', 'True') & hasValue('bumperE', 'False')

# rules = open(behaviourFilePath).read()
# pyDatalog.load(rules)
print("Learning finished")

def eternity():
    print('Loop started')
    
    while(True):
        patience = True
        while(patience):
          try:
              inFile = open('query.txt')
              patience = False
          except:
              print("Query file not found, waiting...")
              time.sleep(0.1)

        # Keep data from file
        inFile = open('query.txt')
        fact = []
        line = inFile.readline()
        factList = ast.literal_eval(line)
        inFile.close()

        print('FactList: ', factList)
        + perceptionBumper(factList) #  Asserts a new fact
        print("New knowledge taken")
        print("Taking a decision...")
        response = str(pyDatalog.ask("takeDecision(D)"))
        print("Decision taken")
        # print(response)
        cleanResponse = response.replace(' ', '').replace('{(', '').replace(',)}', '')
        cleanResponseList = cleanResponse.split(',),(')
        print('Clean Response List: ', cleanResponseList)
        result = cleanResponseList[0]
        - perceptionBumper(factList) #  Retract the fact

        # Put data on file
        outFile = open("response.txt", "w")
        outFile.write(result + '\n')
        outFile.close()
        
        os.remove('query.txt')

eternity()

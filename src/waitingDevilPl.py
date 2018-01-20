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
from pyswip import Prolog
import logging
import ast


behaviourFilePath = 'behaviour.pl'

logging.basicConfig(level=logging.INFO)
prologEngine = Prolog() # The engine given by Pyswip SWI Prolog library

print("Learning started...")
prologEngine.consult(behaviourFilePath)
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
        try:
            factList = ast.literal_eval(line)
            inFile.close()
            fact = 'perceptionBumper(' + str(factList) + ')'.replace("True", "'True'").replace("False", "'False'").replace('"', "'")
            print('Fact: ', fact)
            prologEngine.assertz(fact)
            print("New knowledge taken")
            print("Taking a decision...")
            response = list(prologEngine.query('takeDecision(D)'))
            print("Decision taken")
            print(response)
            result = str(response[0]['D'])
            print('Result: ', result)
            prologEngine.retractall('perceptionBumper(_)')
        except:
            print('Bad data as input')
            result = 'Stay'

        # Put data on file
        outFile = open("response.txt", "w")
        outFile.write(result + '\n')
        outFile.close()
        
        os.remove('query.txt')

eternity()

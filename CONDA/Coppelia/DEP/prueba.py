# -*- coding: utf-8 -*-
"""
Created on Thu Apr 29 22:13:55 2021

@author: Enrique Benavides
"""

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')
    
import time
import sys

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim

if clientID!=-1:
    print ('Connected to remote API server')
else:
    print ('Failed connecting to remote API server')
    sys.exit('No connection')

# res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)

# if res==sim.simx_return_ok:
#     print ('Number of objects in the scene: ',len(objs))
# else:
#     print ('Remote API function call returned with error code: ',res)
    
errorCode,leftMotor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
errorCode,rightMotor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)
errorCode,robot1 = sim.simxGetObjectHandle(clientID, 'lumibot_Frame', sim.simx_opmode_oneshot_wait)


# errorCode,posRob = sim.simxGetObjectPosition(clientID, robot1, -1, sim.simx_opmode_blocking)
# errorCode,oriRob = sim.simxGetObjectOrientation(clientID, robot1, -1, sim.simx_opmode_blocking)
# print("Robot Position: " + str(posRob))
# print("Robot Orientation: " + str(oriRob))

# time.sleep(1)

# sim.simxSetJointTargetVelocity(clientID, leftMotor, 1, sim.simx_opmode_streaming)
# sim.simxSetJointTargetVelocity(clientID, rightMotor, 1, sim.simx_opmode_streaming)

# time.sleep(1)

# sim.simxSetJointTargetVelocity(clientID, leftMotor, 0, sim.simx_opmode_oneshot)
# sim.simxSetJointTargetVelocity(clientID, rightMotor, 0, sim.simx_opmode_oneshot)

errorCode,posRob = sim.simxGetObjectPosition(clientID, robot1, -1, sim.simx_opmode_blocking)
errorCode,oriRob = sim.simxGetObjectOrientation(clientID, robot1, -1, sim.simx_opmode_blocking)
print("Robot Position: " + str(posRob))
print("Robot Orientation: " + str(oriRob))

print ('Program ended')
time.sleep(1)
sim.simxFinish(-1)
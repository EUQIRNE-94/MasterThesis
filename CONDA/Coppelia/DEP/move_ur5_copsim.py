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

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim

if clientID!=-1:
    print ('Connected to remote API server')


    res,UR5_joint1 =sim.simxGetObjectHandle( clientID, 'UR5_joint1', sim.simx_opmode_blocking)
    res,UR5_joint2 =sim.simxGetObjectHandle( clientID, 'UR5_joint2', sim.simx_opmode_blocking)
    res,UR5_joint3 =sim.simxGetObjectHandle( clientID, 'UR5_joint3', sim.simx_opmode_blocking)

    UR5 = [UR5_joint1, UR5_joint2, UR5_joint3]
    
    from math import pi
    print(pi)
   
    UR5_q = []
    for joint in UR5:
        res, value = sim.simxGetJointPosition(clientID, joint, sim.simx_opmode_oneshot)
        print('La posición de: ', joint, ' es ', value)
        UR5_q.append(value)
    
    steps = 100
    
    for t in range(steps):
        for joint in UR5:
            sim.simxSetJointTargetPosition(clientID, joint, t*(pi/2)/steps, sim.simx_opmode_streaming)
        time.sleep(2/steps)
    
    time.sleep(1)
    for t in range(steps,0,-1):
        for joint in UR5:
            sim.simxSetJointTargetPosition(clientID, joint, t*(pi/2)/steps, sim.simx_opmode_streaming)
        time.sleep(2/steps)

        
    time.sleep(2)

    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!\nWe are running Python 3 code.',sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
    
    
else:
    print ('Failed connecting to remote API server')
print ('Program ended')

# -*- coding: utf-8 -*-
"""
Created on Wed May 12

@author: enrique benavides
"""

import sim
import sys
import time
import numpy as np

N = 4

##### Conexión con Coppelia #####
sim.simxFinish(-1)                                                  # Cerrar conexiones existentes
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5)          # Conectarse a Coppelia

if clientID!=-1:
    print ("Iniciar conexion con servidor API")
else:
    print ("Fallo de conexion")
    sys.exit('Saliendo')


ROBOT = ["lumibot_Frame"]
ROBOT_LM = ["lumibot_leftMotor"]
ROBOT_RM = ["lumibot_rightMotor"]
for i in range(N-1):
    ROBOT.append("lumibot_Frame#" + str(i))
    ROBOT_LM.append("lumibot_leftMotor#" + str(i+1))
    ROBOT_RM.append("lumibot_rightMotor#" + str(i+1))

RAMxx = []
RAMxx_LM = []
RAMxx_RM = []
for i in range(N):
    eCode, LM = sim.simxGetObjectHandle(clientID, ROBOT_LM[i], sim.simx_opmode_oneshot_wait)
    RAMxx_LM.append(LM)
    eCode, RM = sim.simxGetObjectHandle(clientID, ROBOT_RM[i], sim.simx_opmode_oneshot_wait)
    RAMxx_RM.append(RM)
    eCode, RAM = sim.simxGetObjectHandle(clientID, ROBOT[i], sim.simx_opmode_oneshot_wait)
    RAMxx.append(RAM)
    
X0 = []
Y0 = []
Xi0 = []
for i in range(N):
    eCode, pos_par = sim.simxGetObjectPosition(clientID, RAMxx[i], -1, sim.simx_opmode_streaming)
    eCode, ori_par = sim.simxGetObjectOrientation(clientID, RAMxx[i], -1, sim.simx_opmode_streaming)
    time.sleep(0.2)
    eCode, pos_par = sim.simxGetObjectPosition(clientID, RAMxx[i], -1, sim.simx_opmode_buffer)
    eCode, ori_par = sim.simxGetObjectOrientation(clientID, RAMxx[i], -1, sim.simx_opmode_buffer)
    
    X0 = np.append(X0,pos_par[0])
    Y0 = np.append(Y0,pos_par[1])
    Xi0 = np.append(Xi0,ori_par[2])
    
print ('Cerrar conexion con servidor API')
time.sleep(1)
sim.simxFinish(-1)
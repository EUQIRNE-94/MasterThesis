# -*- coding: utf-8 -*-
"""
Created on Mon May 10 13:49:40 2021

@author: enrique benavides
"""

import sim
import sys
import time

print("Iniciando Programa")

##### Variables Fijas #####
vd = 1
vi = 0.5

##### Conexión con Coppelia #####
sim.simxFinish(-1)                                          # Cerrar conexiones existentes
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5)  # Conectarse a Coppelia

if clientID!=-1:
    print ("Iniciar conexion con servidor API")
else:
    print ("Fallo de conexion")
    sys.exit('Saliendo')
    
#### Configurar Simulacion ####
sim.simxSynchronous(clientID, True)


#### Obtener Object Handles ####
eCode, RAM01_LM = sim.simxGetObjectHandle(clientID, 'lumibot_leftMotor', sim.simx_opmode_oneshot_wait)
eCode, RAM01_RM = sim.simxGetObjectHandle(clientID, 'lumibot_rightMotor', sim.simx_opmode_oneshot_wait)
eCode, RAM01 = sim.simxGetObjectHandle(clientID, 'lumibot_Frame', sim.simx_opmode_oneshot_wait)

eCode, pos_par = sim.simxGetObjectPosition(clientID, RAM01, -1, sim.simx_opmode_blocking)
eCode, ori_par = sim.simxGetObjectOrientation(clientID, RAM01, -1, sim.simx_opmode_blocking)

print(pos_par)
print(ori_par)

sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)

time.sleep(5)

sim.simxSetJointTargetVelocity(clientID, RAM01_RM, vd, sim.simx_opmode_streaming)
sim.simxSetJointTargetVelocity(clientID, RAM01_LM, vi, sim.simx_opmode_streaming)

for i in range(100):
    eCode, pos_par = sim.simxGetObjectPosition(clientID, RAM01, -1, sim.simx_opmode_blocking)
    eCode, ori_par = sim.simxGetObjectOrientation(clientID, RAM01, -1, sim.simx_opmode_blocking)
    sim.simxSynchronousTrigger(clientID)
    time.sleep(0.001)

time.sleep(0.5)

sim.simxSetJointTargetVelocity(clientID, RAM01_RM, 0, sim.simx_opmode_oneshot)
sim.simxSetJointTargetVelocity(clientID, RAM01_LM, 0, sim.simx_opmode_oneshot)

sim.simxSynchronousTrigger(clientID)

time.sleep(5)

sim.simxPauseSimulation(clientID, sim.simx_opmode_oneshot)
eCode, pos_par = sim.simxGetObjectPosition(clientID, RAM01, -1, sim.simx_opmode_blocking)
eCode, ori_par = sim.simxGetObjectOrientation(clientID, RAM01, -1, sim.simx_opmode_blocking)
time.sleep(5)
sim.simxPauseSimulation(clientID, sim.simx_opmode_oneshot)

time.sleep(5)

print(pos_par)
print(ori_par)

sim.simxStopSimulation(clientID, False)
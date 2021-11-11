# -*- coding: utf-8 -*-
"""
Created on Sat May  1 13:44:07 2021

@author: Enrique Benavides

Prueba de open loop con 1 Lumibot en Coppelia Sim
"""

import sim
import time
import sys
import numpy as np
import funciones as fx
import matplotlib.pyplot as plt

print("Iniciando Programa")

##### Variables Fijas #####
#### Variables de movimiento ####
V0 = 0.20;                                       # Velocidad lineal m/s
rho0 = 0.5                                       # Radio de giro 
w0 = V0/rho0;                                    # Frecuencia natural
    
#### Partículas en el plano ####
N = 1;
M = N;
R0 = 0 + 0*1j;
kappa = 0.1*w0;
A = np.ones([N,N]);
    
#### Variables del robot ####
r_llanta = 0.06/2
L = 0.0886

vd = (2*V0 + w0*L)/(2*r_llanta)
vi = (2*V0 - w0*L)/(2*r_llanta)

#### Variables de tiempo ####
tf = 30
dt = 0.1
tact = 0; tant = 0

#### Variables para graficar ####
x = np.zeros((1,N)); y = np.zeros((1,N)); xi = np.zeros((1,N))
Xr = np.zeros((1,N)); Yr = np.zeros((1,N)); Xir = np.zeros((1,N))
r = np.zeros((1,N),dtype=complex); theta = np.zeros((1,N))
t = [];

##### Conexión con Coppelia #####
sim.simxFinish(-1)                                          # Cerrar conexiones existentes
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5)  # Conectarse a Coppelia

if clientID!=-1:
    print ("Iniciar conexion con servidor API")
else:
    print ("Fallo de conexion")
    sys.exit('Saliendo')
    
##### Posicion inicial #####

#### Obtener Object Handles ####
eCode, RAM01_LM = sim.simxGetObjectHandle(clientID, 'lumibot_leftMotor', sim.simx_opmode_oneshot_wait)
eCode, RAM01_RM = sim.simxGetObjectHandle(clientID, 'lumibot_rightMotor', sim.simx_opmode_oneshot_wait)
eCode, RAM01 = sim.simxGetObjectHandle(clientID, 'lumibot_Frame', sim.simx_opmode_oneshot_wait)

eCode, pos_par = sim.simxGetObjectPosition(clientID, RAM01, -1, sim.simx_opmode_blocking)
eCode, ori_par = sim.simxGetObjectOrientation(clientID, RAM01, -1, sim.simx_opmode_blocking)

X0 = pos_par[0]; Y0 = pos_par[1]; Xi0 = ori_par[2]

#### Sistema ####
print("Iniciando sistema")
t0 = time.time()
t = np.append(t,tact)

x[0,:] = X0
y[0,:] = Y0
xi[0,:] = Xi0

Xr[0,:] = X0
Yr[0,:] = Y0
Xir[0,:] = Xi0

r0 = X0 + Y0*1j
theta0 = Xi0
r_ant = np.zeros((1,N),dtype = complex)
theta_ant = np.zeros((1,N))
r_ant[0,:] = r0
theta_ant[0,:] = theta0
    
r[0,:] = r0
theta[0,:] = theta0

estado = 0

while tact <= tf:
    tact = time.time() - t0
    
    if estado == 0:
        sim.simxSetJointTargetVelocity(clientID, RAM01_RM, vd, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, RAM01_LM, vi, sim.simx_opmode_streaming)
        estado = 1
        
    if tact - tant >= dt:
        ### Particulas en el plano
        dtt = tact - tant
        u_par = np.array([w0])
        [r_act,theta_act] = fx.particlesPlane(V0,r_ant,theta_ant,u_par,dtt)
        
        r = np.append(r,r_act)
        theta = np.append(theta,theta_act)
        
        theta_ant = theta_act
        r_ant = r_act
        
        Xr_par = r_act.real
        Yr_par = r_act.imag
        Xir_par = theta_act
        
        Xr = np.append(Xr,Xr_par)
        Yr = np.append(Yr,Yr_par)
        Xir = np.append(Xir,Xir_par)
        
        ### Uniciclo ###
        eCode, pos_par = sim.simxGetObjectPosition(clientID, RAM01, -1, sim.simx_opmode_blocking)
        eCode, ori_par = sim.simxGetObjectOrientation(clientID, RAM01, -1, sim.simx_opmode_blocking)
        
        x_par = pos_par[0]; y_par = pos_par[1]; xi_par = ori_par[2]
        t = np.append(t,tact)
        x = np.append(x,x_par)
        y = np.append(y,y_par)
        xi = np.append(xi,xi_par)
        
        tant = tact


sim.simxSetJointTargetVelocity(clientID, RAM01_LM, 0, sim.simx_opmode_oneshot)
sim.simxSetJointTargetVelocity(clientID, RAM01_RM, 0, sim.simx_opmode_oneshot)

print ('Cerrar conexion con servidor API')
time.sleep(1)
sim.simxFinish(-1)


#### Impresión de variables ####
plt.close('all')

fig, axs = plt.subplots(3)
fig.suptitle('Estados')
axs[0].grid()
axs[1].grid()
axs[2].grid()
axs[0].plot(t,x,'r-')
axs[0].plot(t,Xr,'k-')
axs[1].plot(t,y, 'r-')
axs[1].plot(t,Yr, 'k-')
axs[2].plot(t,xi,'r-')
axs[2].plot(t,Xir,'k-')

fig = plt.figure(figsize=(10,15))
plt.grid()
plt.title('Plano Fase')
plt.plot(Xr,Yr,'k-')
plt.plot(x,y,'r-')

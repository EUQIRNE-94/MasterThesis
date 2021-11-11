# -*- coding: utf-8 -*-
"""
Created on Sat May  1 13:44:07 2021

@author: Enrique Benavides

Prueba de open loop con 1 Lumibot en Coppelia Sim
"""

import sim
import sys
import time
import numpy as np
import funciones as fx
import matplotlib.pyplot as plt
from math import pi

print("Iniciando Programa")

##### Variables Fijas #####
#### Variables de movimiento ####
V0 = 0.15;                                       # Velocidad lineal m/s
rho0 = 0.5                                       # Radio de giro 
w0 = V0/rho0;                                    # Frecuencia natural
    
#### Partículas en el plano ####
N = 1;
M = N;
R0 = 0 + 0*1j;
kappa = 10*w0;
A = np.ones([N,N]);
    
#### Variables del robot ####
r_llanta = 0.06/2
L = 0.0886

Ks = np.array([[0.8,0,0.75,1,0,1]])  # [Cx,Cxk,Cxi,Cy,Cyk,K]

#### Variables de tiempo ####
# tf = (1/w0)*2*pi
tf = 30
dt = 0.05
n = round(tf/dt)
tspan = np.linspace(0,tf,n)

#### Variables para graficar ####
x = np.zeros((1,N)); y = np.zeros((1,N)); xi = np.zeros((1,N))
Xr = np.zeros((1,N)); Yr = np.zeros((1,N)); Xir = np.zeros((1,N))
err_x = np.zeros((1,N)); err_y = np.zeros((1,N)); err_xi = np.zeros((1,N))
v = np.zeros((1,N)); w = np.zeros((1,N))
r = np.zeros((1,N),dtype=complex); theta = np.zeros((1,N))

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
sim.simxSynchronous(clientID, True)

eCode, RAM01_LM = sim.simxGetObjectHandle(clientID, 'lumibot_leftMotor', sim.simx_opmode_oneshot_wait)
eCode, RAM01_RM = sim.simxGetObjectHandle(clientID, 'lumibot_rightMotor', sim.simx_opmode_oneshot_wait)
eCode, RAM01 = sim.simxGetObjectHandle(clientID, 'lumibot_Frame', sim.simx_opmode_oneshot_wait)

time.sleep(1)

eCode, pos_par = sim.simxGetObjectPosition(clientID, RAM01, -1, sim.simx_opmode_streaming)
eCode, ori_par = sim.simxGetObjectOrientation(clientID, RAM01, -1, sim.simx_opmode_streaming)

time.sleep(1)

eCode, pos_par = sim.simxGetObjectPosition(clientID, RAM01, -1, sim.simx_opmode_buffer)
eCode, ori_par = sim.simxGetObjectOrientation(clientID, RAM01, -1, sim.simx_opmode_buffer)

X0 = pos_par[0]; Y0 = pos_par[1]; Xi0 = ori_par[2]

#### Sistema ####
print("Iniciando sistema")

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

sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)

for k in range(n-1):
    ### Particulas en el plano
    # u_par = np.array([w0])
    u_par = fx.steeringControl(V0,w0,r_ant,theta_ant,R0,kappa,N,M,A)
    [r_act,theta_act] = fx.particlesPlane(V0,r_ant,theta_ant,u_par,dt)
    
    r = np.append(r,r_act)
    theta = np.append(theta,theta_act)
    
    theta_ant = theta_act
    r_ant = r_act
        
    ### Referencias ###
    Xr_par = r_act.real
    Yr_par = r_act.imag
    Xir_par = theta_act
    
    Xr = np.append(Xr,Xr_par)
    Yr = np.append(Yr,Yr_par)
    Xir_par = np.mod(Xir_par,(2*np.pi))
    if Xir_par > np.pi:
        Xir_par -= 2*np.pi
    Xir = np.append(Xir,Xir_par)
    
    ### Uniciclo ###
    eCode, pos_par = sim.simxGetObjectPosition(clientID, RAM01, -1, sim.simx_opmode_buffer)
    eCode, ori_par = sim.simxGetObjectOrientation(clientID, RAM01, -1, sim.simx_opmode_buffer)
    
    x_par = pos_par[0]; y_par = pos_par[1]; xi_par = ori_par[2]
    x = np.append(x,x_par)
    y = np.append(y,y_par)
    xi = np.append(xi,xi_par)
    
    ### Errores ###
    err_xi_par = Xir_par - xi_par
    if np.abs(err_xi_par) > np.pi:
        err_xi_par = err_xi_par - np.sign(err_xi_par)*2*np.pi
    
    err_x_par =  np.cos(xi_par)*(Xr_par - x_par) + np.sin(xi_par)*(Yr_par - y_par);
    err_y_par = -np.sin(xi_par)*(Xr_par - x_par) + np.cos(xi_par)*(Yr_par - y_par);
    
    err_xi = np.append(err_xi,err_xi_par)
    err_x = np.append(err_x,err_x_par)
    err_y = np.append(err_y,err_y_par)
    
    #### Control Uniciclo ####
    [v_par,w_par] = fx.trackingControl(V0,fx.etheta(theta_act,V0),u_par,err_xi_par,err_x_par,err_y_par,Ks,A)
    if v_par < 0:
        v_par = 0
    
    v = np.append(v,v_par)
    w = np.append(w,w_par)
    
    ### Velocidades angulares ###
    vd = (2*v_par + w_par*L)/(2*r_llanta)
    vi = (2*v_par - w_par*L)/(2*r_llanta)
    
    if estado == 0:
        sim.simxSetJointTargetVelocity(clientID, RAM01_RM, np.round(vd,4), sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, RAM01_LM, np.round(vi,4), sim.simx_opmode_streaming)
        estado = 1
    if estado == 1:
        sim.simxSetJointTargetVelocity(clientID, RAM01_RM, np.round(vd,4), sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(clientID, RAM01_LM, np.round(vi,4), sim.simx_opmode_oneshot)
    
    sim.simxSynchronousTrigger(clientID)
    print(tspan[k])
    # time.sleep(0.001)


sim.simxSetJointTargetVelocity(clientID, RAM01_LM, 0, sim.simx_opmode_oneshot)
sim.simxSetJointTargetVelocity(clientID, RAM01_RM, 0, sim.simx_opmode_oneshot)

print ('Cerrar conexion con servidor API')
sim.simxStopSimulation(clientID, False)
time.sleep(1)
sim.simxFinish(-1)



#### Impresión de variables ####
plt.close('all')

fig, axs = plt.subplots(3,figsize=(12,10))
fig.suptitle('Estados')
axs[0].grid()
axs[0].plot(tspan,x,'r--')
axs[0].plot(tspan,Xr,'k-')
axs[0].set(xlabel = "t [s]", ylabel = "x [m]")

axs[1].grid()
axs[1].plot(tspan,y, 'r--')
axs[1].plot(tspan,Yr, 'k-')
axs[1].set(xlabel = "t [s]", ylabel = "y [m]")

axs[2].grid()
axs[2].plot(tspan,xi,'r--')
axs[2].plot(tspan,Xir,'k-')
axs[2].set(xlabel = "t [s]", ylabel = "xi [rad]")

fig, axs = plt.subplots(3,figsize=(12,10))
fig.suptitle('Errores')
axs[0].grid()
axs[1].grid()
axs[2].grid()
axs[0].plot(tspan,err_x,'g-')
axs[0].set(xlabel = "t [s]", ylabel = "error x [m]")
axs[1].plot(tspan,err_y, 'g-')
axs[1].set(xlabel = "t [s]", ylabel = "error y [m]")
axs[2].plot(tspan,err_xi,'g-')
axs[2].set(xlabel = "t [s]", ylabel = "error xi [rad]")

fig, axs = plt.subplots(2,figsize = (12,10))
fig.suptitle('Salidas')
axs[0].grid()
axs[1].grid()
axs[0].set_title("Velocidad lineal")
axs[0].plot(tspan,v,'g-')
axs[0].set(xlabel = "t [s]", ylabel = "[m/s]")
axs[1].set_title("Velocidad angular")
axs[1].plot(tspan,w, 'g-')
axs[1].set(xlabel = "t [s]", ylabel = "[rad/s]")

fig = plt.figure(figsize=(10,10))
plt.grid()
plt.title('Plano Fase')
plt.plot(Xr,Yr,'k-')
plt.plot(x,y,'r--')
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
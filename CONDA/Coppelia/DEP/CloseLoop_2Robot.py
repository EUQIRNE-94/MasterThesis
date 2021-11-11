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
from math import pi

print("Iniciando Programa")

##### Variables Fijas #####
#### Variables de movimiento ####
V0 = 0.15;                                       # Velocidad lineal m/s
rho0 = 0.5                                       # Radio de giro 
w0 = V0/rho0;                                    # Frecuencia natural
    
#### Partículas en el plano ####
N = 2;
M = N;
R0 = 0 + 0*1j;
kappa = 40*w0;
A = np.ones([N,N]);
    
#### Variables del robot ####
r_llanta = 0.06/2
L = 0.0886
d1 = 0.075;
d2 = 0.5;

Ks = np.array([[0.8,0.1,0.75,1,0.1,1]])  # [Cx,Cxk,Cxi,Cy,Cyk,K]

#### Variables de tiempo ####
# tf = (1/w0)*2*pi
tf = 30
dt = 0.1
tact = 0
tant = 0

#### Variables para graficar ####
x = np.zeros((1,N)); y = np.zeros((1,N)); xi = np.zeros((1,N))
Xr = np.zeros((1,N)); Yr = np.zeros((1,N)); Xir = np.zeros((1,N))
err_x = np.zeros((1,N)); err_y = np.zeros((1,N)); err_xi = np.zeros((1,N))
v = np.zeros((1,N));v_eva = np.zeros((1,N)); w = np.zeros((1,N))
r = np.zeros((1,N),dtype=complex); theta = np.zeros((1,N))
t = [];

##### Conexión con Coppelia #####
sim.simxFinish(-1)                                                  # Cerrar conexiones existentes
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5)          # Conectarse a Coppelia

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

eCode, RAM02_LM = sim.simxGetObjectHandle(clientID, 'lumibot_leftMotor#0', sim.simx_opmode_oneshot_wait)
eCode, RAM02_RM = sim.simxGetObjectHandle(clientID, 'lumibot_rightMotor#0', sim.simx_opmode_oneshot_wait)
eCode, RAM02 = sim.simxGetObjectHandle(clientID, 'lumibot_Frame#0', sim.simx_opmode_oneshot_wait)

time.sleep(2)

eCode, pos_par01 = sim.simxGetObjectPosition(clientID, RAM01, -1, sim.simx_opmode_streaming)
eCode, ori_par01 = sim.simxGetObjectOrientation(clientID, RAM01, -1, sim.simx_opmode_streaming)
eCode, pos_par02 = sim.simxGetObjectPosition(clientID, RAM02, -1, sim.simx_opmode_streaming)
eCode, ori_par02 = sim.simxGetObjectOrientation(clientID, RAM02, -1, sim.simx_opmode_streaming)

time.sleep(2)

eCode, pos_par01 = sim.simxGetObjectPosition(clientID, RAM01, -1, sim.simx_opmode_buffer)
eCode, ori_par01 = sim.simxGetObjectOrientation(clientID, RAM01, -1, sim.simx_opmode_buffer)
eCode, pos_par02 = sim.simxGetObjectPosition(clientID, RAM02, -1, sim.simx_opmode_buffer)
eCode, ori_par02 = sim.simxGetObjectOrientation(clientID, RAM02, -1, sim.simx_opmode_buffer)

X0 = np.array([pos_par01[0],pos_par02[0]])
Y0 = np.array([pos_par01[1],pos_par02[1]])
Xi0 = np.array([ori_par01[2],ori_par02[2]])

# sys.exit()

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
        
    if tact - tant >= dt:
        ### Particulas en el plano
        dtt = tact - tant
        u_par = fx.steeringControl(V0,w0,r_ant,theta_ant,R0,kappa,N,M,A)
        [r_act,theta_act] = fx.particlesPlane(V0,r_ant,theta_ant,u_par,dtt)
        
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
        
        for k in range(N):
            if Xir_par[:,k] > np.pi:
                Xir_par[:,k] -= 2*np.pi
                
        Xir = np.append(Xir,Xir_par)
        
        ### Uniciclo ###
        eCode, pos_par01 = sim.simxGetObjectPosition(clientID, RAM01, -1, sim.simx_opmode_buffer)
        eCode, ori_par01 = sim.simxGetObjectOrientation(clientID, RAM01, -1, sim.simx_opmode_buffer)
        eCode, pos_par02 = sim.simxGetObjectPosition(clientID, RAM02, -1, sim.simx_opmode_buffer)
        eCode, ori_par02 = sim.simxGetObjectOrientation(clientID, RAM02, -1, sim.simx_opmode_buffer)
        
        x_par = np.array([pos_par01[0],pos_par02[0]])
        y_par = np.array([pos_par01[1],pos_par02[1]])
        xi_par = np.array([ori_par01[2],ori_par02[2]])
        
        t = np.append(t,tact)
        x = np.append(x,x_par)
        y = np.append(y,y_par)
        xi = np.append(xi,xi_par)
        
        ### Errores ###
        err_xi_par = Xir_par - xi_par
        for k in range(N):
            if np.abs(err_xi_par[:,k]) > np.pi:
                err_xi_par[:,k] = err_xi_par[:,k] - np.sign(err_xi_par[:,k])*2*np.pi
            
        err_x_par =  np.cos(xi_par)*(Xr_par - x_par) + np.sin(xi_par)*(Yr_par - y_par);
        err_y_par = -np.sin(xi_par)*(Xr_par - x_par) + np.cos(xi_par)*(Yr_par - y_par);
            
        err_xi = np.append(err_xi,err_xi_par)
        err_x = np.append(err_x,err_x_par)
        err_y = np.append(err_y,err_y_par)
        
        #### Control Uniciclo ####
        [v_par,w_par] = fx.trackingControl(V0,fx.etheta(theta_act,V0),u_par,err_xi_par,err_x_par,err_y_par,Ks,A)
        eva_p = fx.evasion(x_par,y_par,d1,d2)
        v_eva_par = eva_p*v_par
        
        for k in range(N):
            if v_par[k] < 0:
                v_par[k] = 0
            if v_eva_par[k] < 0:
                v_eva_par[k] = 0
        
            
        v = np.append(v,v_par)
        v_eva = np.append(v_eva,v_eva_par)
        w = np.append(w,w_par)
        
        ### Velocidades angulares ###
        # vd = (2*v_par + w_par*L)/(2*r_llanta)
        # vi = (2*v_par - w_par*L)/(2*r_llanta)
        vd = (2*v_eva_par + w_par*L)/(2*r_llanta)
        vi = (2*v_eva_par - w_par*L)/(2*r_llanta)
        
        if estado == 0:
            sim.simxSetJointTargetVelocity(clientID, RAM01_RM, np.round(vd[0],4), sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, RAM01_LM, np.round(vi[0],4), sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, RAM02_RM, np.round(vd[1],4), sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, RAM02_LM, np.round(vi[1],4), sim.simx_opmode_streaming)
            estado = 1
        if estado == 1:
            sim.simxSetJointTargetVelocity(clientID, RAM01_RM, np.round(vd[0],4), sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(clientID, RAM01_LM, np.round(vi[0],4), sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(clientID, RAM02_RM, np.round(vd[1],4), sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(clientID, RAM02_LM, np.round(vi[1],4), sim.simx_opmode_oneshot)
        
        tant = tact
        
        print(round(tact,4))
        


sim.simxSetJointTargetVelocity(clientID, RAM01_LM, 0, sim.simx_opmode_oneshot)
sim.simxSetJointTargetVelocity(clientID, RAM01_RM, 0, sim.simx_opmode_oneshot)
sim.simxSetJointTargetVelocity(clientID, RAM02_LM, 0, sim.simx_opmode_oneshot)
sim.simxSetJointTargetVelocity(clientID, RAM02_RM, 0, sim.simx_opmode_oneshot)

print ('Cerrar conexion con servidor API')
time.sleep(1)
sim.simxFinish(-1)

#### Reacomodo de variables ####
x = x.reshape((-1,N))
y = y.reshape((-1,N))
xi = xi.reshape((-1,N))

Xr = Xr.reshape((-1,N))
Yr = Yr.reshape((-1,N))
Xir = Xir.reshape((-1,N))

err_x = err_x.reshape((-1,N))
err_y = err_y.reshape((-1,N))
err_xi = err_xi.reshape((-1,N))

v = v.reshape((-1,N))
v_eva = v_eva.reshape((-1,N))
w = w.reshape((-1,N))

#### Impresión de variables ####
labels = ["RAM01","RAM02"]
labels_r = ["RAM01r","RAM02r"]
color = ['r','g','b','c','m','y','k']
color_r = ['r--','g--','b--','c--','m--','y--','k--']
plt.close('all')


##### Estados #####
fig, axs = plt.subplots(3,figsize=(15,10))
fig.suptitle('Estados')
axs[0].grid()
axs[1].grid()
axs[2].grid()

for k in range(N):
    axs[0].plot(t,x[:,k], color[k])
    axs[0].plot(t,Xr[:,k], color_r[k])
    axs[1].plot(t,y[:,k], color[k])
    axs[1].plot(t,Yr[:,k], color_r[k])
    axs[2].plot(t,xi[:,k], color[k], label = labels[k])
    axs[2].plot(t,Xir[:,k], color_r[k], label = labels_r[k])

axs[0].set(xlabel = "t [s]", ylabel = "x [m]")
axs[1].set(xlabel = "t [s]", ylabel = "y [m]")
axs[2].set(xlabel = "t [s]", ylabel = "xi [rad]")
axs[2].legend(bbox_to_anchor = (0.5,0), loc = "lower center", bbox_transform = fig.transFigure, ncol = N)

##### Errores #####
fig, axs = plt.subplots(3,figsize=(15,10))
fig.suptitle('Errores')
axs[0].grid()
axs[1].grid()
axs[2].grid()

for k in range(N):
    axs[0].plot(t,err_x[:,k], color[k])
    axs[1].plot(t,err_y[:,k], color[k])
    axs[2].plot(t,err_xi[:,k], color[k], label = labels[k])

axs[0].set(xlabel = "t [s]", ylabel = "error x [m]")
axs[1].set(xlabel = "t [s]", ylabel = "error y [m]")
axs[2].set(xlabel = "t [s]", ylabel = "error xi [rad]")
axs[2].legend(bbox_to_anchor = (0.5,0), loc = "lower center", bbox_transform = fig.transFigure, ncol = N)

##### Salidas #####
fig, axs = plt.subplots(3,figsize = (15,10))
fig.suptitle('Salidas')
axs[0].grid()
axs[1].grid()
axs[2].grid()
axs[0].set_title("Velocidad lineal")
axs[1].set_title("Velocidad lineal con evasion")
axs[2].set_title("Velocidad angular")

for k in range(N):
    axs[0].plot(t,v[:,k], color[k])
    axs[1].plot(t,v_eva[:,k], color[k])
    axs[2].plot(t,w[:,k], color[k], label = labels[k])

axs[0].set(xlabel = "t [s]", ylabel = "[m/s]")
axs[1].set(xlabel = "t [s]", ylabel = "[m/s]")
axs[2].set(xlabel = "t [s]", ylabel = "[rad/s]")
axs[2].legend(bbox_to_anchor = (0.5,0), loc = "lower center", bbox_transform = fig.transFigure, ncol = N)

##### Plano Fase #####
fig = plt.figure(figsize=(10,10))
plt.grid()
plt.title('Plano Fase')

for k in range(N):
    plt.plot(Xr[:,k],Yr[:,k], color[k], label = labels_r[k])
    plt.plot(x[:,k],y[:,k], color_r[k], label = labels[k])

plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend(bbox_to_anchor = (0.5,0), loc = "lower center", bbox_transform = fig.transFigure, ncol = N)
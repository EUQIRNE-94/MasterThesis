# -*- coding: utf-8 -*-
"""
18 de mayo del 2021

@author: enrique benavides

"Movimiento Colectivo de un Sistema Multi-Agente Usando las Ecuaciones planares de Frenet-Serret"
 
Formación simétrica -> linea
"""

import sim
import time
import sys
import numpy as np
import funciones_cop as fx
from math import pi

print("Iniciando Programa")

########## Variables Fijas ##########

#### Variables de movimiento ####
V0 = 0.15 # Velocidad lineal m/s
rho0 = 2 # Radio de giro 
w0 = V0/rho0 # Frecuencia natural

#### Partículas en el plano ####
N = 8 # Cantidad de particulas
M = N # Cantidad de subgrupos
R0 = 0 + 0*1j # Centro de giro
kappa = (1/V0)*w0 # Ganancia de sistema
A = np.ones([N,N]) # Matriz de comunicación

#### Variables del robot ####
r_llanta = 0.06/2 # Radio de llanta
L = 0.0886 # Distancia entre llantas
d1 = 0.0886 # Radio de robot
d2 = 0.25 # Radio de seguridad entre robots para evitar colisión
Ks = np.array([[1.5,0.15,0.1,15,0.15,1]])  # [Cx,Cxk,Cxi,Cy,Cyk,K]
ang_f = 180/N # Angulo de separación entre uniciclos en formacion linea

#### Variables de tiempo ####
# tf = (1/w0)*2*pi
tf = 150
dt = 0.1
n = round(tf/dt)
tspan = np.linspace(0,tf,n)

#### Variables para graficar ####
x = np.zeros((1,N)); y = np.zeros((1,N)); xi = np.zeros((1,N)) # Estados de uniciclo
x_par = []; y_par = []; xi_par = [] # Estados parciales

Xr = np.zeros((1,N)); Yr = np.zeros((1,N)); Xir = np.zeros((1,N)) # Referencias espaciales
Xr_par = np.zeros((1,N)); Yr_par = np.zeros((1,N)); Xir_par = np.zeros((1,N))

err_x = np.zeros((1,N)); err_y = np.zeros((1,N)); err_xi = np.zeros((1,N)) # Errores
v = np.zeros((1,N));v_eva = np.zeros((1,N)); w = np.zeros((1,N)) # Salidas de control
v_r = np.ones((1,N))*V0; w_r = np.ones((1,N))*w0


########## Conexión con Coppelia ##########
print("Conexion con Coppelia")
sim.simxFinish(-1)                                                  # Cerrar conexiones existentes
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5)          # Conectarse a Coppelia

if clientID!=-1:
    print ("Iniciar conexion con servidor API")
else:
    print ("Fallo de conexion")
    sys.exit('Saliendo')

########## Posiciones iniciales ##########

#### Obtener Object Handles ####
print("Posición Inicial")
sim.simxSynchronous(clientID, True) # Iniciar modo sincrono con coppelia

ROBOT = ["lumibot_Frame"]; ROBOT_LM = ["lumibot_leftMotor"]; ROBOT_RM = ["lumibot_rightMotor"] # Nombres de los 'Handles'
RAMxx = []; RAMxx_LM = []; RAMxx_RM = [] # Números de 'Handles'
X0 = []; Y0 = []; Xi0 = [] # Vector de posicion inicial

for i in range(N-1):
    ROBOT.append("lumibot_Frame#" + str(i))
    ROBOT_LM.append("lumibot_leftMotor#" + str(i))
    ROBOT_RM.append("lumibot_rightMotor#" + str(i))

for i in range(N):
    eCode, LM = sim.simxGetObjectHandle(clientID, ROBOT_LM[i], sim.simx_opmode_oneshot_wait)
    RAMxx_LM.append(LM)
    eCode, RM = sim.simxGetObjectHandle(clientID, ROBOT_RM[i], sim.simx_opmode_oneshot_wait)
    RAMxx_RM.append(RM)
    eCode, RAM = sim.simxGetObjectHandle(clientID, ROBOT[i], sim.simx_opmode_oneshot_wait)
    RAMxx.append(RAM)
    time.sleep(0.5)
    
for i in range(N):
    eCode, pos_par = sim.simxGetObjectPosition(clientID, RAMxx[i], -1, sim.simx_opmode_streaming)
    eCode, ori_par = sim.simxGetObjectOrientation(clientID, RAMxx[i], -1, sim.simx_opmode_streaming)
    time.sleep(0.2)
    eCode, pos_par = sim.simxGetObjectPosition(clientID, RAMxx[i], -1, sim.simx_opmode_buffer)
    eCode, ori_par = sim.simxGetObjectOrientation(clientID, RAMxx[i], -1, sim.simx_opmode_buffer)
    
    X0 = np.append(X0,pos_par[0])
    Y0 = np.append(Y0,pos_par[1])
    Xi0 = np.append(Xi0,ori_par[2])

########## Sistema ##########
print("Iniciando Sistema")

r0 = X0 + Y0*1j; theta0 = Xi0 # Posición inicial en partículas
x[0,:] = X0; y[0,:] = Y0; xi[0,:] = Xi0 # Posición inicial en estados de uniciclo
Xr[0,:] = X0; Yr[0,:] = Y0; Xir[0,:] = Xi0 # Posición inicial en referencias de uniciclo

sim.simxStartSimulation(clientID, sim.simx_opmode_blocking) # Inicial simulación

# Habilitar el modo de envío a Coppelia como Stream
for i in range(N):
    sim.simxSetJointTargetVelocity(clientID, RAMxx_RM[i], 0, sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(clientID, RAMxx_LM[i], 0, sim.simx_opmode_streaming)
    time.sleep(0.1)

# Simulación del sistema
for k in range(n-1):
    ### Particulas en el plano ###
    
    # Generar el control de partícula e integrar su movimiento
    ### Referencias ###
        
    for i in range(N):
        Xir_par[0,i] = (V0/rho0)*(k*dt)+(i*2*pi)/N
        Xr_par = rho0*np.cos(Xir_par)
        Yr_par = rho0*np.sin(Xir_par)
            
    Xir_par = np.mod(Xir_par,(2*np.pi))
    for kk in range(N):
        if Xir_par[:,kk] > np.pi:
            Xir_par[:,kk] -= 2*np.pi
    
    Xr = np.append(Xr,Xr_par); Yr = np.append(Yr,Yr_par) # Guardar estados de referencia X,Y
    Xir = np.append(Xir,Xir_par) # Guardar estado de referencia de orientación
    x_par = []; y_par = []; xi_par = [] # Limpiar estados parciales
    
    # Obtener posición del robot
    for i in range(N):
        eCode, pos_par = sim.simxGetObjectPosition(clientID, RAMxx[i], -1, sim.simx_opmode_buffer)
        eCode, ori_par = sim.simxGetObjectOrientation(clientID, RAMxx[i], -1, sim.simx_opmode_buffer)
    
        x_par = np.append(x_par,pos_par[0])
        y_par = np.append(y_par,pos_par[1])
        xi_par = np.append(xi_par,ori_par[2])
    
    x = np.append(x,x_par); y = np.append(y,y_par); xi = np.append(xi,xi_par) # Guardar estados de uniciclo
    
    ### Errores ###
    err_xi_par = Xir_par - xi_par
    err_xi_par = err_xi_par.flatten()
    for kk in range(N):
        if np.abs(err_xi_par[kk]) > np.pi:
            err_xi_par[kk] = err_xi_par[kk] - np.sign(err_xi_par[kk])*2*np.pi
    
    err_x_par =  np.cos(xi_par)*(Xr_par - x_par) + np.sin(xi_par)*(Yr_par - y_par);
    err_y_par = -np.sin(xi_par)*(Xr_par - x_par) + np.cos(xi_par)*(Yr_par - y_par);
    
    err_xi = np.append(err_xi,err_xi_par); err_x = np.append(err_x,err_x_par); err_y = np.append(err_y,err_y_par) # Guardar errores de seguimiento
    
    #### Control Uniciclo ####
    # Generar el control de seguimiento para el uniciclo
    [v_par,w_par] = fx.trackingControl2(V0,v_r,w_r,err_xi_par,err_x_par,err_y_par,Ks,A)
    # [v_par,w_par] = fx.trackingControl(V0,fx.etheta(theta_act,V0),u_par,err_xi_par,err_x_par,err_y_par,Ks,A)
    # Evasión de colisiones de los robots
    eva_p = fx.evasion(x_par,y_par,d1,d2)
    v_eva_par = eva_p*v_par
    
    for kk in range(N):
        if v_par[kk] < 0:
            v_par[kk] = 0
        if v_eva_par[kk] < 0:
            v_eva_par[kk] = 0
    
    v = np.append(v,v_par); v_eva = np.append(v_eva,v_eva_par); w = np.append(w,w_par) # Guardar salidas de control de seguimiento
    
    ### Velocidades angulares ###
    # vd = (2*v_par + w_par*L)/(2*r_llanta)
    # vi = (2*v_par - w_par*L)/(2*r_llanta)
    vd = (2*v_eva_par + w_par*L)/(2*r_llanta)
    vi = (2*v_eva_par - w_par*L)/(2*r_llanta)
    
    # Enviar velocidades angulares al robot
    for i in range(N):
        sim.simxSetJointTargetVelocity(clientID, RAMxx_RM[i], np.round(vd[i],2), sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(clientID, RAMxx_LM[i], np.round(vi[i],2), sim.simx_opmode_oneshot)
    
    sim.simxSynchronousTrigger(clientID) # Realizar el pasod e integración
    print("Time: " + str(tspan[k]))

for i in range(N):
    sim.simxSetJointTargetVelocity(clientID, RAMxx_LM[i], 0, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, RAMxx_RM[i], 0, sim.simx_opmode_oneshot)

print ('Cerrar conexion con servidor API')
sim.simxStopSimulation(clientID, False)
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

########## Guardar información en archivo ##########
np.savez('DATA/01b_data',robots=N,tiempo=tspan,Xref=Xr,Yref=Yr,Xiref=Xir,xest=x,yest=y,xiest=xi,errx=err_x,erry=err_y,errxi=err_xi,vellin=v,velang=w,veleva=v_eva)
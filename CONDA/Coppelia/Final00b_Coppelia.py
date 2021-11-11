# -*- coding: utf-8 -*-
"""
18 de mayo del 2021

@author: enrique benavides

"Movimiento Colectivo de un Sistema Multi-Agente Usando las Ecuaciones planares de Frenet-Serret"
 
Formación simétrica
Evasion colisiones
Comunicación limitada
"""

import sim
import time
import sys
import numpy as np
import funciones as fx
import matplotlib.pyplot as plt

print("Iniciando Programa")

########## Variables Fijas ##########

#### Variables de movimiento ####
V0 = 0.15 # Velocidad lineal m/s
rho0 = 1 # Radio de giro 
w0 = V0/rho0 # Frecuencia natural

#### Partículas en el plano ####
N = 6 # Cantidad de particulas
M = N # Cantidad de subgrupos
R0 = 0 + 0*1j # Centro de giro
kappa = (1/V0)*w0 # Ganancia de sistema
A = np.ones([N,N]) # Matriz de comunicación

#### Variables del robot ####
r_llanta = 0.06/2 # Radio de llanta
L = 0.0886 # Distancia entre llantas
d1 = 0.0886 # Radio de robot
d2 = 0.25 # Radio de seguridad entre robots para evitar colisión
Ks = np.array([[0.8,0.1,0.75,1,0.1,1]])  # [Cx,Cxk,Cxi,Cy,Cyk,K] Ganancias de control del uniciclo

### Matriz de comunicacion uniciclos ###
Acom1 = fx.anillo(N);
# Acom2 = fx.anilloB(N);

perturbacion = False
if perturbacion == True:
    t_ini = 45
    t_fin = 50
    lista = list(range(N))
    part_pert = np.random.choice(lista,2)
    print(part_pert)

#### Variables de tiempo ####
# tf = (1/w0)*2*pi
tf = 180
dt = 0.1
n = round(tf/dt)
tspan = np.linspace(0,tf,n)

#### Variables para graficar ####
r = np.zeros((1,N),dtype=complex); theta = np.zeros((1,N)) # Estados partículas
r_ant = np.zeros((1,N),dtype = complex); theta_ant = np.zeros((1,N)) # Estados para integración de particulas
c = np.zeros((1,N),dtype = complex) # Centro de particulas
x = np.zeros((1,N)); y = np.zeros((1,N)); xi = np.zeros((1,N)) # Estados de uniciclo
Xr = np.zeros((1,N)); Yr = np.zeros((1,N)); Xir = np.zeros((1,N)) # Referencias espaciales
err_x = np.zeros((1,N)); err_y = np.zeros((1,N)); err_xi = np.zeros((1,N)) # Errores
v = np.zeros((1,N));v_eva = np.zeros((1,N)); w = np.zeros((1,N)) # Salidas de control
x_par = []; y_par = []; xi_par = [] # Estados parciales

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
r_ant[0,:] = r0; theta_ant[0,:] = theta0 # Posición inicial para el integrador
r[0,:] = r0; theta[0,:] = theta0 # Posicion inicial en estados de partícula
x[0,:] = X0; y[0,:] = Y0; xi[0,:] = Xi0 # Posición inicial en estados de uniciclo
Xr[0,:] = X0; Yr[0,:] = Y0; Xir[0,:] = Xi0 # Posición inicial en referencias de uniciclo
c[0,:] = r0 + (1j/w0)*fx.etheta(theta0, V0) # Posición inicial del centro de giro de partícula

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
    u_par = fx.steeringControl(V0,w0,r_ant,theta_ant,R0,kappa,N,M,A)
    [r_act,theta_act] = fx.particlesPlane(V0,r_ant,theta_ant,u_par,dt)
    c_act = r_act + (1j/w0)*fx.etheta(theta_act, V0) # Obtener centro de giro de partícula
    
    r = np.append(r,r_act); theta = np.append(theta,theta_act) # Guardar estados de partícula
    c = np.append(c,c_act) # Guardar posición de centro de giro de partícula
    theta_ant = theta_act; r_ant = r_act # Actualizar posición para integrar partícula
    
    ### Referencias ###
    Xr_par = r_act.real; Yr_par = r_act.imag; Xir_par = theta_act # Generar partículas de referencia
    Xr = np.append(Xr,Xr_par); Yr = np.append(Yr,Yr_par) # Guardar estados de referencia X,Y
    
    Xir_par = np.mod(Xir_par,(2*np.pi))
    for kk in range(N):
        if Xir_par[:,kk] > np.pi:
            Xir_par[:,kk] -= 2*np.pi
    
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
    for kk in range(N):
        if np.abs(err_xi_par[:,kk]) > np.pi:
            err_xi_par[:,kk] = err_xi_par[:,kk] - np.sign(err_xi_par[:,kk])*2*np.pi
    
    err_x_par =  np.cos(xi_par)*(Xr_par - x_par) + np.sin(xi_par)*(Yr_par - y_par);
    err_y_par = -np.sin(xi_par)*(Xr_par - x_par) + np.cos(xi_par)*(Yr_par - y_par);
    
    err_xi = np.append(err_xi,err_xi_par); err_x = np.append(err_x,err_x_par); err_y = np.append(err_y,err_y_par) # Guardar errores de seguimiento
    
    #### Control Uniciclo ####
    # Generar el control de seguimiento para el uniciclo
    [v_par,w_par] = fx.trackingControl(V0,fx.etheta(theta_act,V0),u_par,err_xi_par,err_x_par,err_y_par,Ks,Acom1)
    # Evasión de colisiones de los robots
    eva_p = fx.evasion(x_par,y_par,d1,d2)
    v_eva_par = eva_p*v_par
    
    if perturbacion == True and tspan[k] > t_ini and tspan[k] < t_fin:
        for kk in range(part_pert.size):
            v_par[part_pert[kk]] = 0
            v_eva_par[part_pert[kk]] = 0
            w_par[part_pert[kk]] = 0
    
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
c = c.reshape((-1,N))

Xr = Xr.reshape((-1,N))
Yr = Yr.reshape((-1,N))
Xir = Xir.reshape((-1,N))

err_x = err_x.reshape((-1,N))
err_y = err_y.reshape((-1,N))
err_xi = err_xi.reshape((-1,N))

v = v.reshape((-1,N))
v_eva = v_eva.reshape((-1,N))
w = w.reshape((-1,N))

########## Gráficos ##########

# #### Impresión de variables ####
# labels = []
# labels_r = []
# for i in range(N):
#     labels.append("RAM"+str(i+1))
#     labels_r.append("RAM"+str(i+1)+"r")
    
# plt.close('all')

# ##### Estados #####
# fig, axs = plt.subplots(3,figsize=(15,10))
# fig.suptitle('Estados')
# axs[0].grid()
# axs[1].grid()
# axs[2].grid()

# for k in range(N):
#     axs[0].plot(tspan,x[:,k])
#     axs[0].plot(tspan,Xr[:,k], '--')
#     axs[1].plot(tspan,y[:,k])
#     axs[1].plot(tspan,Yr[:,k], '--')
#     axs[2].plot(tspan,xi[:,k], label = labels[k])
#     axs[2].plot(tspan,Xir[:,k], '--', label = labels_r[k])

# axs[0].set(xlabel = "t [s]", ylabel = "x [m]")
# axs[1].set(xlabel = "t [s]", ylabel = "y [m]")
# axs[2].set(xlabel = "t [s]", ylabel = "xi [rad]")
# axs[2].legend(bbox_to_anchor = (0.5,0), loc = "lower center", bbox_transform = fig.transFigure, ncol = N)

# ##### Salidas #####
# fig, axs = plt.subplots(3,figsize = (15,10))
# fig.suptitle('Salidas')
# axs[0].grid()
# axs[1].grid()
# axs[2].grid()
# axs[0].set_title("Velocidad lineal")
# axs[1].set_title("Velocidad lineal con evasion")
# axs[2].set_title("Velocidad angular")

# for k in range(N):
#     axs[0].plot(tspan,v[:,k])
#     axs[1].plot(tspan,v_eva[:,k])
#     axs[2].plot(tspan,w[:,k], label = labels[k])

# axs[0].set(xlabel = "t [s]", ylabel = "[m/s]")
# axs[1].set(xlabel = "t [s]", ylabel = "[m/s]")
# axs[2].set(xlabel = "t [s]", ylabel = "[rad/s]")
# axs[2].legend(bbox_to_anchor = (0.5,0), loc = "lower center", bbox_transform = fig.transFigure, ncol = N)

# ##### Plano Fase #####
# fig = plt.figure(figsize=(10,10))
# plt.grid()
# plt.title('Plano Fase')

# for k in range(N):
#     plt.plot(x[:,k],y[:,k], label = labels[k])
#     plt.plot(Xr[:,k],Yr[:,k], '--', label = labels_r[k])

# plt.xlabel("X [m]")
# plt.ylabel("Y [m]")
# plt.legend(bbox_to_anchor = (0.5,0), loc = "lower center", bbox_transform = fig.transFigure, ncol = N)

# ##### Errores #####
# fig, axs = plt.subplots(3,figsize=(15,10))
# fig.suptitle('Errores')
# axs[0].grid()
# axs[1].grid()
# axs[2].grid()

# for k in range(N):
#     axs[0].plot(tspan,err_x[:,k])
#     axs[1].plot(tspan,err_y[:,k])
#     axs[2].plot(tspan,err_xi[:,k], label = labels[k])

# axs[0].set(xlabel = "t [s]", ylabel = "error x [m]")
# axs[1].set(xlabel = "t [s]", ylabel = "error y [m]")
# axs[2].set(xlabel = "t [s]", ylabel = "error xi [rad]")
# axs[2].legend(bbox_to_anchor = (0.5,0), loc = "lower center", bbox_transform = fig.transFigure, ncol = N)

# ##### Error con centro de giro #####
# err_c = R0 - c
# err_cc = np.sqrt(err_c.real*err_c.real + err_c.imag*err_c.imag)

# fig = plt.figure(figsize=(15,10))
# plt.grid()
# plt.title('Error centros')

# for k in range(N):
#     plt.plot(tspan,err_cc[:,k], label = labels[k])

# plt.xlabel("Tiempo [s]")
# plt.ylabel("Error [m]")
# plt.legend(bbox_to_anchor = (0.5,0), loc = "lower center", bbox_transform = fig.transFigure, ncol = N)

########## Guardar información en archivo ##########
if perturbacion == True:
    np.savez('DATA/00b_dataPert',robots=N,tiempo=tspan,Xref=Xr,Yref=Yr,Xiref=Xir,xest=x,yest=y,xiest=xi,errx=err_x,erry=err_y,errxi=err_xi,vellin=v,velang=w,veleva=v_eva)
else:
    np.savez('DATA/00b_data',robots=N,tiempo=tspan,Xref=Xr,Yref=Yr,Xiref=Xir,xest=x,yest=y,xiest=xi,errx=err_x,erry=err_y,errxi=err_xi,vellin=v,velang=w,veleva=v_eva)
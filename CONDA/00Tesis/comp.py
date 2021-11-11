# -*- coding: utf-8 -*-
"""
@author: enrique benavides

 "Movimiento Colectivo de un Sistema Multi-Agente Usando las Ecuaciones planares de Frenet-Serret"

 Seguimiento de particulas a formación simétrica

 Modelo de partículas de Sepulchre [0-02] eq.(37)
 Seguimiento de uniciclos planteado por Morales
 Evasión de colisiones en uniciclo
"""

import funciones as fx
import numpy as np
from matplotlib import pyplot as plt
from math import pi
import sys

### Semilla para crear elementos aleatorios
seed = np.random.SeedSequence(2)
rng = np.random.default_rng(seed)

## Variables de tiempo
tf = 45
dt = 0.1
n = round(tf/dt)
tspan = np.linspace(0,tf,n)

#### Partículas en el plano ####

# r = v_0/w0;
V0 = 0.15;
r_0 = 2
w0 = V0/r_0;
N = 8;
M = N;
rx = 1.5;
ry = 1.5;

##### Condiciones Iniciales #####

#### Uniciclos ####

i = N;
d1 = 0.075;
d2 = 0.5;
K = 5;
# Ks = np.array([[0.02,0.1,0.005,1,0.1,0.5]])  # [Cx,Cxk,Cxi,Cy,Cyk,K]
# Ks = np.array([[0.15,0.1,0.25,1,0.1,1]])  # [Cx,Cxk,Cxi,Cy,Cyk,K]
Ks = np.array([[1.5,0.15,0.1,15,0.15,1]])  # [Cx,Cxk,Cxi,Cy,Cyk,K]

## Condiciones Iniciales ##
x0 = np.array([1,0,-1, 0,0.75,-0.75, 0.75,-0.75])
y0 = np.array([0,1, 0,-1,0.75,-0.75,-0.75, 0.75])
xi0 = np.array([-pi,-pi/2,0,pi/2,-2.0071,0.6981,2.5307,-0.8726])

##### Integración del sistema #####

#### Prealocacion de variables ####
### Uniciclos ###
x = np.zeros((n,N)); y = np.zeros((n,N)); xi = np.zeros((n,N))
Xr_par = np.zeros((1,N)); Yr_par = np.zeros((1,N)); Xir_par = np.zeros((1,N))
Xr = np.zeros((n,N)); Yr = np.zeros((n,N)); XIr = np.zeros((n,N))
v_r = np.zeros((n,N)); w_r = np.zeros((n,N))
v = np.zeros((n,N)); w = np.zeros((n,N))
err_x = np.zeros((n,N)); err_y = np.zeros((n,N)); err_xi = np.zeros((n,N))
eva = np.zeros((n,N)); v_eva = np.zeros((n,N))

### Matriz de adyacencia de comunicación partículas ###
A = np.ones([N,N]);
### Matriz de comunicacion uniciclos ###
# Acom1 = anillo(N);
# Acom2 = anilloB(N);

x[0,:] = x0
y[0,:] = y0
xi[0,:] = xi0

for k in range(n-1):
    
    #### Referencias en espacio ####
    for i in range(N):
        Xir_par[0,i] = (V0/r_0)*(k*dt)+(i*2*pi)/N
    Xr_par = r_0*np.cos(Xir_par)
    Yr_par = r_0*np.sin(Xir_par)
    
    Xr[k,:] = Xr_par
    Yr[k,:] = Yr_par
    XIr[k,:] = Xir_par
    
    #### Referencias de velocidad ####
    v_r[k,:] = V0
    w_r[k,:] = w0
    
    #### Errores ####
    err_xi[k,:] = XIr[k,:] - xi[k,:]
    err_x[k,:] =  np.cos(xi[k,:])*(Xr[k,:] - x[k,:]) + np.sin(xi[k,:])*(Yr[k,:] - y[k,:]);
    err_y[k,:] = -np.sin(xi[k,:])*(Xr[k,:] - x[k,:]) + np.cos(xi[k,:])*(Yr[k,:] - y[k,:]);
    
    #### Uniciclos ####
    [v[k,:],w[k,:]] = fx.trackingControl2(V0,v_r[k,:],w_r[k,:],err_xi[k,:],err_x[k,:],err_y[k,:],Ks,A)
    eva[k,:] = fx.evasion(x[k,:],y[k,:],d1,d2,k)
    v_eva[k,:] = eva[k,:]*v[k,:]
    [x[k+1,:],y[k+1,:],xi[k+1,:]] = fx.unicycleModel(x[k,:],y[k,:],xi[k,:],v_eva[k,:],w[k,:],dt)

### Acomodo de variables ###
x = np.delete(x,n-1,0); y = np.delete(y,n-1,0); xi = np.delete(xi,n-1,0); eva = np.delete(eva,n-1,0);
err_x = np.delete(err_x,n-1,0); err_y = np.delete(err_y,n-1,0); err_xi = np.delete(err_xi,n-1,0);
v = np.delete(v,n-1,0); v_eva = np.delete(v_eva,n-1,0); w = np.delete(w,n-1,0);
v_r = np.delete(v_r,n-1,0); w_r = np.delete(w_r,n-1,0);
Xr = np.delete(Xr,n-1,0); Yr = np.delete(Yr,n-1,0); XIr = np.delete(XIr,n-1,0);
tspan = np.delete(tspan,n-1,0);

XIr = np.mod(XIr,2*pi)-pi
xi = np.mod(xi,2*pi)-pi

n = n-2

##### Figuras #####
maxXY = np.max(np.abs([x,y])) + 0.25
plt.close('all')

# ## Simulacion
# plt.figure(figsize = (10,10))

# plt.axis([-maxXY,maxXY,-maxXY,maxXY])

# plt.grid()

# for k in range(0,n-1,10):
#     plt.axis([-maxXY,maxXY,-maxXY,maxXY])
#     plt.scatter(r[k,:].real,r[k,:].imag,marker='o',linewidth=5 ,color='r',zorder=1)
#     plt.scatter(Xr[k,:],Yr[k,:],marker='o',linewidth=1 ,color='g',zorder=2)
#     plt.title('Phase Plane')
#     plt.xlabel(r'$x [m]$')
#     plt.ylabel(r'$y [m]$')
#     plt.grid()
#     plt.pause(0.05)
#     plt.cla()

# plt.close('all')

## Plano Fase
plt.figure(figsize = (10,10))

# plt.axis([-maxXY,maxXY,-maxXY,maxXY])
plt.plot(Xr,Yr,'k',linewidth=4,zorder=1)
plt.plot(x,y,'g--',zorder=1)
for k in range(i+1):
    point = fx.unicycleFigure(x[n,k], y[n,k], xi[n,k]+pi, 0.1)
    plt.fill(point[0,:],point[1,:],facecolor='blue',zorder=3)
plt.title('Phase Plane')
plt.xlabel(r'$x [m]$')
plt.ylabel(r'$y [m]$')
plt.grid()

## Estados

plt.figure(figsize = (18,10))

plt.subplot(3,1,1)
plt.title('Xr')
plt.plot(tspan,Xr)
plt.plot(tspan,x,'--')
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'$Xr [m]$')
plt.grid()

plt.subplot(3,1,2)
plt.title('Yr')
plt.plot(tspan,Yr)
plt.plot(tspan,y,'--')
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'$Yr [m]$')
plt.grid()

plt.subplot(3,1,3)
plt.title('Theta')
plt.plot(tspan,XIr)
plt.plot(tspan,xi,'--')
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'$THETA [rad]$')
plt.grid()

## Errores
plt.figure(figsize = (18,10))

plt.subplot(3,1,1)
plt.plot(tspan,err_x)
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'$Error X [m]$')
plt.grid()

plt.subplot(3,1,2)
plt.plot(tspan,err_y)
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'$Error Y [m]$')
plt.grid()

plt.subplot(3,1,3)
plt.plot(tspan,err_xi)
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'$Error XI [m]$')
plt.grid()

## Entradas de control
plt.figure(figsize = (18,10))

plt.subplot(2,1,1)
plt.title('Angular Velocity')
plt.plot(tspan,w_r)
plt.plot(tspan,w)
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'$w [rad/sec]$')
plt.grid()

plt.subplot(2,1,2)
plt.title('Linear Velocity with Evasion')
plt.plot(tspan,v_r)
plt.plot(tspan,v_eva)
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'$v [m/sec]$')
plt.grid()


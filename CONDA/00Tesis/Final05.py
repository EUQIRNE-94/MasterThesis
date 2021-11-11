# -*- coding: utf-8 -*-
"""
Created on Thu Feb 18 15:06:54 2021

@author: enrique benavides

 "Movimiento Colectivo de un Sistema Multi-Agente Usando las Ecuaciones planares de Frenet-Serret"

 Seguimiento de particulas a formación simétrica -> Formación en linea ->
 Formación "v"

 Modelo de partículas de Sepulchre [0-02] eq.(37)
 Seguimiento de uniciclos planteado por Morales
 Evasión de colisiones en uniciclo
 Comunicación limitada en uniciclo
"""

import funciones as fx
import numpy as np
from matplotlib import pyplot as plt

### Semilla para crear elementos aleatorios
seed = np.random.SeedSequence(2)
rng = np.random.default_rng(seed)

## Variables de tiempo
tf = 100
dt = 0.05
n = round(tf/dt)
tspan = np.linspace(0,tf,n)


#### Partículas en el plano ####

# r = v_0/w0;
V0 = 1;
r_0 = 2;
w0 = V0/r_0;
N = 8;
M = N;
R0 = 0 + 0*1j;
kappa = w0;
rx = 5;
ry = 5;

##### Condiciones Iniciales #####

#### Particulas ####

### Aleatorios ###
# r0 = rng.choice(np.linspace(-rango,rango,100),N) + rng.choice(np.linspace(-rango,rango,100),N)*1j
# theta0 = rng.choice(np.linspace(0,2*pi,100),N)
### 6 Particulas ###
# r0 = np.array([[-2.33-0.19*1j,0.30+2.40*1j,1.90-1.71*1j,0.84+1.77*1j,-1.54+0.72*1j,-0.65-0.61*1j]])
# theta0 = np.array([[0.67,6.04,0.02,4.86,5.13,5.45]])
### 8 Partículas ###
r0 = np.array([[4.84+4.39*1j,3.58-1.98*1j,2.85-2.04*1j,0.13-1.67*1j,-3.22-0.32*1j,-1.01+1.48*1j,-3.66-4.74*1j,-4.69+3.42*1j]])
theta0 = np.array([[3.51,5.36,2.18,2.80,0.34,1.11,4.16,2.07]])
### 10 Particulas ###
# r0 = np.array([[-4.14+4.63*1j,-2.37+0.46*1j,3.01+0.21*1j,-4.78-2.68*1j,4.28-0.11*1j,2.30+1.24*1j,-0.11+1.79*1j,0.78-1.04*1j,-2.62-1.32*1j,-0.41+4.87*1j]])
# theta0 = np.array([[0.23,5.56,5.73,5.00,0.62,1.64,2.11,4.27,0.85,4.53]])

#### Uniciclos ####

i = N;
d1 = 0.075;
d2 = 0.5;
K = 5;
Ks = np.array([[5,0.1,0.5,1,0.1,0.5]])  # [Cx,Cxk,Cxi,Cy,Cyk,K]

## Condiciones Iniciales ##
x0 = np.real(r0) + rng.choice(np.linspace(-0.05,0.05,25),N)*np.real(r0)
y0 = np.imag(r0) + rng.choice(np.linspace(-0.05,0.05,25),N)*np.imag(r0)
xi0 = theta0 + rng.choice(np.linspace(-0.05,0.05,25),N)*theta0

##### Integración del sistema #####

#### Prealocacion de variables ####
### Particulas en el plano ###
r = np.zeros((n,N),dtype = complex); theta = np.zeros((n,N)); u = np.zeros((n,N))
c = np.zeros((n,N)); R = np.zeros((n,N))

### Uniciclos ###
x = np.zeros((n,N)); y = np.zeros((n,N)); xi = np.zeros((n,N))
Xr = np.zeros((n,N)); Yr = np.zeros((n,N)); XIr = np.zeros((n,N))
v = np.zeros((n,N)); w = np.zeros((n,N))
err_x = np.zeros((n,N)); err_y = np.zeros((n,N)); err_xi = np.zeros((n,N))
eva = np.zeros((n,N)); v_eva = np.zeros((n,N))

### Condiciones Iniciales de Integración ###
r[0,:] = r0; theta[0,:] = theta0
x[0,:] = x0; y[0,:] = y0; xi[0,:] = xi0

### Matriz de adyacencia de comunicación partículas ###
A = np.ones([N,N]);
### Matriz de comunicacion uniciclos ###
Acom1 = fx.anillo(N);
Acom2 = fx.anilloB(N);

ang_f = 180/i

for k in range(n-1):
    #### Particulas en el plano ####
    u[k,:] = fx.steeringControl(V0,w0,r[k,:],theta[k,:],R0,kappa,N,M,A)
    [r[k+1,:],theta[k+1,:]] = fx.particlesPlane(V0,r[k,:],theta[k,:],u[k,:],dt)
    
    #### Coordenadas de referencia ####
    if tspan[k] < 50:
        Xr[k,:] = r[k,:].real
        Yr[k,:] = r[k,:].imag
        XIr[k,:] = theta[k,:]
    else:
        if tspan[k] < 75:
            [Xr[k,:],Yr[k,:],XIr[k,:]] = fx.lineaForm(V0,w0,r[k,:],theta[k,:],R0,1,ang_f,i)
        else:
            [Xr[k,:],Yr[k,:],XIr[k,:]] = fx.puntaForm(r[k,:],theta[k,:],R0,1,150,0.75,i)
            d2 = 0.25
    
    #### Errores ####
    err_xi[k,:] = XIr[k,:] - xi[k,:]
    err_x[k,:] =  np.cos(xi[k,:])*(Xr[k,:] - x[k,:]) + np.sin(xi[k,:])*(Yr[k,:] - y[k,:]);
    err_y[k,:] = -np.sin(xi[k,:])*(Xr[k,:] - x[k,:]) + np.cos(xi[k,:])*(Yr[k,:] - y[k,:]);
    
    #### Uniciclos ####
    [v[k,:],w[k,:]] = fx.trackingControl(V0,fx.etheta(theta[k,:],V0),u[k,:],err_xi[k,:],err_x[k,:],err_y[k,:],Ks,Acom2)
    eva[k,:] = fx.evasion(x[k,:],y[k,:],d1,d2,k)
    v_eva[k,:] = eva[k,:]*v[k,:]
    [x[k+1,:],y[k+1,:],xi[k+1,:]] = fx.unicycleModel(x[k,:],y[k,:],xi[k,:],v_eva[k,:],w[k,:],dt)

### Acomodo de variables ###
r = np.delete(r,n-1,0); theta = np.delete(theta,n-1,0); u = np.delete(u,n-1,0);
R = np.delete(R,n-1,0); c = np.delete(c,n-1,0);
x = np.delete(x,n-1,0); y = np.delete(y,n-1,0); xi = np.delete(xi,n-1,0); eva = np.delete(eva,n-1,0);
err_x = np.delete(err_x,n-1,0); err_y = np.delete(err_y,n-1,0); err_xi = np.delete(err_xi,n-1,0);
v = np.delete(v,n-1,0); v_eva = np.delete(v_eva,n-1,0); w = np.delete(w,n-1,0);
Xr = np.delete(Xr,n-1,0); Yr = np.delete(Yr,n-1,0); XIr = np.delete(XIr,n-1,0);
tspan = np.delete(tspan,n-1,0);

n = n-1

##### Figuras #####
maxXY = np.max(np.abs([x,y])) + 0.25
plt.close('all')

## Simulacion
plt.figure(figsize = (10,10))

plt.axis([-maxXY,maxXY,-maxXY,maxXY])

plt.grid()

for k in range(0,n-1,3):
    plt.axis([-maxXY,maxXY,-maxXY,maxXY])
    plt.scatter(r[k,:].real,r[k,:].imag,marker='o',linewidth=5 ,color='r',zorder=2)
    plt.scatter(Xr[k,:],Yr[k,:],marker='o',linewidth=1 ,color='g',zorder=3)
    for l in range(i):
        point = fx.unicycleFigure(x[k,l], y[k,l], xi[k,l], 0.25)
        plt.fill(point[0,:],point[1,:],facecolor='blue',zorder=1)
    plt.title('Phase Plane')
    plt.xlabel(r'$x [m]$')
    plt.ylabel(r'$y [m]$')
    plt.grid()
    plt.pause(0.01)
    plt.cla()

plt.close('all')

## Plano Fase
plt.figure(figsize = (10,10))

plt.axis([-maxXY,maxXY,-maxXY,maxXY])
plt.plot(r.real,r.imag,'k',linewidth=4,zorder=1)
plt.plot(x,y,'g--',zorder=1)
plt.scatter(r[n-1,:].real,r[n-1,:].imag,marker='o' ,color='r',zorder=4)
for k in range(i):
    point = fx.unicycleFigure(x[n-1,k], y[n-1,k], xi[n-1,k], 0.25)
    plt.fill(point[0,:],point[1,:],facecolor='blue',zorder=3)
plt.title('Phase Plane')
plt.xlabel(r'$x [m]$')
plt.ylabel(r'$y [m]$')
plt.grid()

## Errores
plt.figure(figsize = (18,10))

plt.subplot(3,1,1)
plt.title('Error in X')
plt.plot(tspan,err_x)
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'$Error X [m]$')
plt.grid()

plt.subplot(3,1,2)
plt.title('Error in Y')
plt.plot(tspan,err_y)
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'$Error Y [m]$')
plt.grid()

plt.subplot(3,1,3)
plt.title('Error in xi')
plt.plot(tspan,err_xi)
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'$Error XI [m]$')
plt.grid()

## Entradas de control
plt.figure(figsize = (18,10))

plt.subplot(2,2,1)
plt.title('Control Input')
plt.plot(tspan,u)
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'$u [rad/sec]$')
plt.grid()

plt.subplot(2,2,2)
plt.title('Evasion')
plt.plot(tspan,eva)
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'$Evasion$')
plt.grid()

plt.subplot(2,2,3)
plt.title('Angular Velocity')
plt.plot(tspan,w)
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'$w [rad/sec]$')
plt.grid()

plt.subplot(2,2,4)
plt.title('Linear Velocity with Evasion')
plt.plot(tspan,v_eva)
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'$v [m/sec]$')
plt.grid()
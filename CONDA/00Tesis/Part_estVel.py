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

### Semilla para crear elementos aleatorios
seed = np.random.SeedSequence(2)
rng = np.random.default_rng(seed)

## Variables de tiempo
tf = 50
dt = 0.1
n = round(tf/dt)
tspan = np.linspace(0,tf,n)

#### Partículas en el plano ####

# r = v_0/w0;
V0 = 1;
r_0 = 0.5
w0 = V0/r_0;
N = 6;
M = N;
R0 = 0 + 0*1j;
kappa = w0
rx = 5;
ry = 5;

##### Condiciones Iniciales #####

#### Particulas ####

### Aleatorios ###
r0 = rng.choice(np.linspace(-rx,rx,100),N) + rng.choice(np.linspace(-ry,ry,100),N)*1j
theta0 = rng.choice(np.linspace(0,2*pi,100),N)

### 2 Particulas ###
# r0 = np.array([[-0.945+0.6*1j]])
# theta0 = np.array([[4.01425]])
### 2 Particulas ###
# r0 = np.array([[-0.945+0.6*1j,1.225+0.45*1j]])
# theta0 = np.array([[4.01425,2.26895]])
# ### 3 Particulas ###
# r0 = np.array([[-0.945+0.6*1j,1.225+0.45*1j,-0.35-1.15*1j]])
# theta0 = np.array([[4.01425,2.26895,0.34907]])
### 4 Particulas ###
# r0 = np.array([[-0.575-0.2999*1j,0.5999+0.2250*1j]])
# theta0 = np.array([[0.5235,1.7453]])
### 6 Particulas ###
# r0 = np.array([[-2.33-0.19*1j,0.30+2.40*1j,1.90-1.71*1j,0.84+1.77*1j,-1.54+0.72*1j,-0.65-0.61*1j]])
# theta0 = np.array([[0.67,6.04,0.02,4.86,5.13,5.45]])
### 8 Partículas ###
# r0 = np.array([[4.84+4.39*1j,3.58-1.98*1j,2.85-2.04*1j,0.13-1.67*1j,-3.22-0.32*1j,-1.01+1.48*1j,-3.66-4.74*1j,-4.69+3.42*1j]])
# theta0 = np.array([[3.51,5.36,2.18,2.80,0.34,1.11,4.16,2.07]])
### 10 Particulas ###
# r0 = np.array([[-4.14+4.63*1j,-2.37+0.46*1j,3.01+0.21*1j,-4.78-2.68*1j,4.28-0.11*1j,2.30+1.24*1j,-0.11+1.79*1j,0.78-1.04*1j,-2.62-1.32*1j,-0.41+4.87*1j]])
# theta0 = np.array([[0.23,5.56,5.73,5.00,0.62,1.64,2.11,4.27,0.85,4.53]])


##### Integración del sistema #####

#### Prealocacion de variables ####
### Particulas en el plano ###
r = np.zeros((n,N),dtype = complex); theta = np.zeros((n,N)); u = np.zeros((n,N))
c = np.zeros((n,N),dtype = complex); R = np.zeros((n,N))

### Condiciones Iniciales de Integración ###
r[0,:] = r0; theta[0,:] = theta0

### Matriz de adyacencia de comunicación partículas ###
A = np.ones([N,N]);

uuu = np.zeros((n,5*N))
rrr = np.zeros((n,5*N),dtype = complex)
ccc = np.zeros((n,5*N),dtype = complex)

for i in range(5):
    V0 = 1- i*0.2;
    w0 = V0/r_0

    for k in range(n-1):
        #### Particulas en el plano ####
        c[k,:] = r[k,:] + (1j/w0)*fx.etheta(theta[k,:],V0)
        u[k,:] = fx.steeringControl(V0,w0,r[k,:],theta[k,:],R0,kappa,N,M,A)
        # u[k,:] = w0*np.ones((1,N))
        [r[k+1,:],theta[k+1,:]] = fx.particlesPlane(V0,r[k,:],theta[k,:],u[k,:],dt)
    
    uuu[:,i*N:i*N+N] = u[:,0:N]
    rrr[:,i*N:i*N+N] = r[:,0:N]
    ccc[:,i*N:i*N+N] = c[:,0:N]
    print(i)
    
plt.figure(figsize = (18,10))
plt.title('Control Input')
plt.plot(tspan[0:n-1],uuu[0:n-1])
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'$u [rad/sec]$')
plt.grid()

### Acomodo de variables ###
r = np.delete(r,n-1,0); theta = np.delete(theta,n-1,0); u = np.delete(u,n-1,0);
R = np.delete(R,n-1,0); c = np.delete(c,n-1,0);

tspan = np.delete(tspan,n-1,0);

n = n-1

theta = np.mod(theta,2*pi) - pi

##### Figuras #####
# maxXY = np.max(np.abs([r.real,r.imag])) + 0.25
# plt.close('all')

## Plano Fase
plt.figure(figsize = (10,10))

# plt.axis([-maxXY,maxXY,-maxXY,maxXY])
plt.plot(rrr.real,rrr.imag,linewidth=2,zorder=1)
plt.scatter(rrr[n-1,:].real,rrr[n-1,:].imag,marker='o' ,color='r',zorder=4)
plt.title('Phase Plane')
plt.xlabel(r'$x [m]$')
plt.ylabel(r'$y [m]$')
plt.grid()

## Entradas de control
# plt.figure(figsize = (18,10))

# plt.title('Control Input')
# plt.plot(tspan,u)
# plt.xlabel(r'$Time [s]$')
# plt.ylabel(r'$u [rad/sec]$')
# plt.grid()

## Error con centro de formacion
err_c = R0 - ccc
err_cc = np.sqrt(err_c.real*err_c.real + err_c.imag*err_c.imag)

plt.figure(figsize = (18,10))
plt.title('Error centro particula')
plt.plot(tspan[0:n-1],err_cc[0:n-1])
plt.xlabel(r'$Time [s]$')
plt.ylabel(r'$centro [m]$')
plt.grid()
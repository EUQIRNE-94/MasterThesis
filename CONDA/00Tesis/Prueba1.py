# -*- coding: utf-8 -*-
"""
Created on Wed Sep 30 19:25:46 2020

@author: Enrique Benavides
"""

import numpy as np
from matplotlib import pyplot as plt
from math import pi

seed = np.random.SeedSequence(2)
rng = np.random.default_rng(seed)

##########          Funciones          ###########
def etheta(theta):
    
    y = np.cos(theta) + np.sin(theta)*1j
    
    return y

def control(w0):
    return w0

## Variables Fijas
w0 = 1
N = 4
K = -w0
rango = 10

## Tiempo
dt = 0.05
rev = 5
tfinal = rev*(2*pi/w0)
n = round(tfinal/dt)
tspan = np.linspace(0,tfinal,n)

## Prealocacion de variables
r = np.zeros((n,N),dtype = complex); theta = np.zeros((n,N)); u = np.zeros((n,N))



r0 = rng.choice(np.linspace(-rango,rango,100),N) + rng.choice(np.linspace(-rango,rango,100),N)*1j
theta0 = rng.choice(np.linspace(0,2*pi,100),N)

r[0,:] = r0
theta[0,:] = theta0

for k in range(n-1):
    r[k+1,:] = r[k,:] + etheta(theta[k,:])*dt;
    u[k,:] = control(w0);
    theta[k+1,:] = theta[k,:] + u[k,:]*dt;
    
plt.close('all')
fig = plt.figure(figsize = (8,8))
plt.plot(r.real,r.imag)
plt.xlabel(r'$x$')
plt.ylabel(r'$y$')

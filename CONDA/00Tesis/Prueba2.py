# -*- coding: utf-8 -*-
"""
Created on Wed Sep 30 19:25:46 2020

@author: Enrique Benavides
"""

import numpy as np
from matplotlib import pyplot as plt
from math import pi

## Variables Fijas
V_0 = 0.15
r_0 = 0.5
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

Xr = np.zeros((n,N)); Yr = np.zeros((n,N)); Xir = np.zeros((n,N))

for k in range(n-1):
    
    #### Referencias en espacio ####
    for i in range(N):
        Xir[k,i] = (V_0/r_0)*(k*dt)+(i*2*pi)/N
    Xr[k,:] = r_0*np.cos(Xir[k,:])
    Yr[k,:] = r_0*np.sin(Xir[k,:])
    
    
plt.close('all')
fig = plt.figure(figsize = (8,8))
plt.plot(Xr[0:n-1,:],Yr[0:n-1,:])
plt.plot(Xr[n-2,:],Yr[n-1,:],'ro')
plt.xlabel(r'$x$')
plt.ylabel(r'$y$')

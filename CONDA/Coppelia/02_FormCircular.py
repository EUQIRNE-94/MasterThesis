# -*- coding: utf-8 -*-
"""
Created on Thu May 20 13:35:50 2021

@author: enriq
"""

import numpy as np
from matplotlib import pyplot as plt
import funciones as fx

data = np.load('DATA/02_data.npz')

N = data['robots']
tspan = data['tiempo']
x = data['xest']
y = data['yest']
xi = data['xiest']
Xr = data['Xref']
Yr = data['Yref']
Xir = data['Xiref']
err_x = data['errx']
err_y = data['erry']
err_xi = data['errxi']
v = data['vellin']
w = data['velang']
v_eva = data['veleva']

#### Impresión de variables ####
n = tspan.size - 1
labels = []
labels_r = []
for i in range(N):
    labels.append("ROB"+str(i+1))
    labels_r.append("ROB"+str(i+1)+"r")
    
plt.close('all')

##### Estados #####
fig, axs = plt.subplots(3,figsize=(15,10))
fig.suptitle('Estados',fontsize=26)
axs[0].grid()
axs[1].grid()
axs[2].grid()

for k in range(N):
    axs[0].plot(tspan,x[:,k])
    axs[0].plot(tspan,Xr[:,k], '--')
    axs[1].plot(tspan,y[:,k])
    axs[1].plot(tspan,Yr[:,k], '--')
    axs[2].plot(tspan,xi[:,k], label = labels[k])
    axs[2].plot(tspan,Xir[:,k], '--', label = labels_r[k])

axs[0].set(xlabel = "t [s]", ylabel = "x [m]")
axs[1].set(xlabel = "t [s]", ylabel = "y [m]")
axs[2].set(xlabel = "t [s]", ylabel = "xi [rad]")
axs[2].legend(bbox_to_anchor = (0.5,0), loc = "lower center", bbox_transform = fig.transFigure, ncol = N)

##### Salidas #####
fig, axs = plt.subplots(2,figsize = (15,10))
fig.suptitle('Velocidades',fontsize=26)
axs[0].grid()
axs[1].grid()
axs[0].set_title("Velocidad Lineal con Evasión")
axs[1].set_title("Velocidad Angular")
axs[0].set_xlim(right=150)
axs[1].set_xlim(right=150)

for k in range(N):
    axs[0].plot(tspan,v_eva[:,k])
    axs[1].plot(tspan,w[:,k], label = labels[k])

axs[0].set(xlabel = "t [s]", ylabel = "[m/s]")
axs[1].set(xlabel = "t [s]", ylabel = "[rad/s]")
axs[1].legend(bbox_to_anchor = (0.5,0), loc = "lower center", bbox_transform = fig.transFigure, ncol = N)

##### Plano Fase #####
fig = plt.figure(figsize=(10,10))
plt.grid()
plt.title('Plano Fase',fontsize=26)

for k in range(N):
    plt.plot(x[:,k],y[:,k], label = labels[k])
    plt.plot(Xr[:,k],Yr[:,k], '--', label = labels_r[k])
    point = fx.unicycleFigure(x[n,k], y[n,k], xi[n,k], 0.1)
    plt.fill(point[0,:],point[1,:],facecolor='blue',zorder=3)
    plt.text(x[n,k],y[n,k],str(k+1))
plt.plot(Xr[n,0],Yr[n,0], 'or',zorder=4)

plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend(bbox_to_anchor = (0.5,0), loc = "lower center", bbox_transform = fig.transFigure, ncol = N)

##### Errores #####
fig, axs = plt.subplots(3,figsize=(15,10))
fig.suptitle('Errores',fontsize=26)
axs[0].grid()
axs[1].grid()
axs[2].grid()

for k in range(N):
    axs[0].plot(tspan[:],err_x[:,k])
    axs[1].plot(tspan[:],err_y[:,k])
    axs[2].plot(tspan[:],err_xi[:,k], label = labels[k])

axs[0].set(xlabel = "t [s]", ylabel = "error x [m]")
axs[1].set(xlabel = "t [s]", ylabel = "error y [m]")
axs[2].set(xlabel = "t [s]", ylabel = "error xi [rad]")
axs[2].legend(bbox_to_anchor = (0.5,0), loc = "lower center", bbox_transform = fig.transFigure, ncol = N)
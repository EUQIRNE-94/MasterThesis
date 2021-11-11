# -*- coding: utf-8 -*-
"""
Created on Thu May 20 13:35:50 2021

@author: enriq
"""

import numpy as np
from matplotlib import pyplot as plt

dataCol = np.load('DATA/00a_data.npz')
data = np.load('DATA/00_data.npz')

## Evasion de colisiones
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

## NO Evasion de colisiones
Nc = dataCol['robots']
tspanc = dataCol['tiempo']
xc = dataCol['xest']
yc = dataCol['yest']
xic = dataCol['xiest']
Xrc = dataCol['Xref']
Yrc = dataCol['Yref']
Xirc = dataCol['Xiref']
err_xc = dataCol['errx']
err_yc = dataCol['erry']
err_xic = dataCol['errxi']
vc = dataCol['vellin']
wc = dataCol['velang']
v_evac = dataCol['veleva']

#### Impresión de variables ####
labels = []
labels_r = []
for i in range(N):
    labels.append("RAM"+str(i+1))
    labels_r.append("RAM"+str(i+1)+"r")
    
plt.close('all')

##### Estados #####
fig, axs = plt.subplots(3,figsize=(15,10))
fig.suptitle('Estados')
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
fig, axs = plt.subplots(3,figsize = (15,10))
fig.suptitle('Salidas')
axs[0].grid()
axs[1].grid()
axs[2].grid()
axs[0].set_title("Velocidad lineal")
axs[1].set_title("Velocidad lineal con evasion")
axs[2].set_title("Velocidad angular")

for k in range(N):
    axs[0].plot(tspan,v[:,k])
    axs[1].plot(tspan,v_eva[:,k])
    axs[2].plot(tspan,w[:,k], label = labels[k])

axs[0].set(xlabel = "t [s]", ylabel = "[m/s]")
axs[1].set(xlabel = "t [s]", ylabel = "[m/s]")
axs[2].set(xlabel = "t [s]", ylabel = "[rad/s]")
axs[2].legend(bbox_to_anchor = (0.5,0), loc = "lower center", bbox_transform = fig.transFigure, ncol = N)

##### Plano Fase #####
fig = plt.figure(figsize=(10,10))
plt.grid()
plt.title('Plano Fase')

for k in range(N):
    plt.plot(x[:,k],y[:,k], label = labels[k])
    plt.plot(Xr[:,k],Yr[:,k], '--', label = labels_r[k])

plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend(bbox_to_anchor = (0.5,0), loc = "lower center", bbox_transform = fig.transFigure, ncol = N)

##### Errores #####
fig, axs = plt.subplots(3,figsize=(15,10))
fig.suptitle('Errores')
axs[0].grid()
axs[1].grid()
axs[2].grid()

for k in range(N):
    axs[0].plot(tspan,err_x[:,k])
    axs[1].plot(tspan,err_y[:,k])
    axs[2].plot(tspan,err_xi[:,k], label = labels[k])

axs[0].set(xlabel = "t [s]", ylabel = "error x [m]")
axs[1].set(xlabel = "t [s]", ylabel = "error y [m]")
axs[2].set(xlabel = "t [s]", ylabel = "error xi [rad]")
axs[2].legend(bbox_to_anchor = (0.5,0), loc = "lower center", bbox_transform = fig.transFigure, ncol = N)
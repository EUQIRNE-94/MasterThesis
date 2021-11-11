# -*- coding: utf-8 -*-
"""
Created on Wed Apr 21 12:43:04 2021

@author: enriq
"""

import serial
import numpy as np
import time
import matplotlib.pyplot as plt

## Diccionarios de los robots ##
robots = dict(ram01 = "COM9", 
              ram02 = "COM11", 
              ram03 = "COM14", 
              ram04 = "COM16", 
              ram05 = "COM17",
              ram   = "COM4")


## Variables ##

salD = []; salI = []; velD = []; velI = []; refD = []; refI = []
t = []
tc = 0; rD = 0; rI = 0

## Conexión con robot ##

try:
    print("RAM")
    RAM = serial.Serial(robots['ram01'],9600)
except:
    print("Sin conexion")

# str0 = "@W23%"
str0 = "@W7%"
RAM.write(str0.encode('ascii'))
# str0 = "@W2191P0.5I1.5D$"
# RAM.write(str0.encode('ascii'))

t0 = time.time()

RAM.read_until()
RAM.read_until()

## Lecturas ##
cambio = 0
try:
    while True:
        tact = time.time() - t0
        
        if (RAM.inWaiting() > 0):
            data = RAM.read_until()
            a2 = data[2:len(data)-2].decode()
            valuesfloat = np.array(a2.split(","),dtype=float)
            salD = np.append(salD,valuesfloat[0])
            salI = np.append(salI,valuesfloat[1])
            velD = np.append(velD,valuesfloat[2])
            velI = np.append(velI,valuesfloat[3])
            refD = np.append(refD,rD)
            refI = np.append(refI,rI)
            t = np.append(t,tact)
        
        if (tact >= 1) and cambio == 0:
            rD = 50; rI = 50
            str1 = "@W" + str(rD) + "D" + str(rI) + "I#"
            RAM.write(str1.encode('ascii'))
            tc = tact
            cambio = 1

        if (tact >= 15) and cambio == 1:
            break
            
except KeyboardInterrupt:
    pass

str2 = "@W0D0I#"
RAM.write(str2.encode('ascii'))

RAM.close()

A = np.array([t,velD,velI])
A = A.T
M = np.array([t,salD,velD,salI,velI])
M = M.T

#### Mostrar informacion
plt.close('all')

fig = plt.figure(figsize=(10,15))
plt.grid()
plt.title('Velocidad Derecha')
# plt.plot(t,refD,'k-')
plt.plot(t,velD, 'r-')
plt.legend(['Ref. Der','Vel.Der'],loc = 'upper left')

fig = plt.figure(figsize=(10,15))
plt.grid()
plt.title('Velocidad Izquierda')
# plt.plot(t,refI,'k-')
plt.plot(t,velI, 'r-')
plt.legend(['Ref.Izq','Vel.Izq'],loc = 'upper left')

fig = plt.figure(figsize=(10,15))
plt.grid()
plt.title('Salidas PWM')
plt.plot(t,salD,'k-')
plt.plot(t,salI, 'r-')
plt.legend(['Salida. Der','Salida.Izq'],loc = 'upper left')
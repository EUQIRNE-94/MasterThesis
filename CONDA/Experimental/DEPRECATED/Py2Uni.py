# -*- coding: utf-8 -*-
"""
Created on Monday March 29 2021

@author: Enrique Benavides
"""

import serial
import numpy as np
import time
import matplotlib.pyplot as plt



robots = dict(ram01 = "COM9", 
              ram02 = "COM11", 
              ram03 = "COM14", 
              ram04 = "COM16", 
              ram05 = "COM17")

refD = []; refI = []; t = []
salD = []; salI = []; velD = []; velI = []

print('RAM01')
RAM01 = serial.Serial(robots['ram01'],9600)
time.sleep(2)

data = RAM01.read_until()
#### Plectura base
data = RAM01.read_until()
a2 = data[0:len(data)-2].decode()
valuesfloat = np.array(a2.split(","),dtype=float)
t0 = time.time()
print(t0)
salD = np.append(salD,valuesfloat[0])
salI = np.append(salI,valuesfloat[1])
velD = np.append(velD,valuesfloat[2])
velI = np.append(velI,valuesfloat[3])

cambio = 0

rd = 0
ri = 0
refD.append(rd)
refI.append(ri)
t.append(0)


try:
    while True:
        if (RAM01.inWaiting() > 0):
            data = RAM01.read_until()
            a2 = data[0:len(data)-2].decode()
            valuesfloat = np.array(a2.split(","),dtype=float)
            print(valuesfloat)
            salD = np.append(salD,valuesfloat[0])
            salI = np.append(salI,valuesfloat[1])
            velD = np.append(velD,valuesfloat[2])
            velI = np.append(velI,valuesfloat[3])
            refD.append(rd)
            refI.append(ri)
            tact = time.time()
            t.append(tact-t0)
        if (tact-t0 >= 5) and cambio == 0:
            cad = "@W10D10I#"
            rd = 10
            ri = 10
            RAM01.write(cad.encode('ascii'))
            cambio = 1
        if (tact-t0 >= 15) and cambio == 1:
            cad = "@W5D5I#"
            rd = 5
            ri = 5
            RAM01.write(cad.encode('ascii'))
            cambio = 2
        if (tact-t0 >= 25) and cambio == 2:
            cad = "@W0D0I#"
            rd = 0
            ri = 0
            RAM01.write(cad.encode('ascii'))
            break
except KeyboardInterrupt:
    pass

RAM01.close()


#### Mostrar informacion
plt.close('all')

fig = plt.figure(figsize=(10,15))
plt.grid()
plt.title('Velocidad Derecha')
plt.plot(t,refD,'k-')
plt.plot(t,velD, 'r-')
plt.legend(['Ref. Der','Vel.Der'],loc = 'upper left')

fig = plt.figure(figsize=(10,15))
plt.grid()
plt.plot(t,refI,'k-')
plt.plot(t,velI, 'r-')
plt.legend(['Ref.Izq','Vel.Izq'],loc = 'upper left')

fig = plt.figure(figsize=(10,15))
plt.grid()
plt.plot(t,salD,'k-')
plt.plot(t,salI, 'r-')
plt.legend(['Salida. Der','Salida.Izq'],loc = 'upper left')




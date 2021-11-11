# -*- coding: utf-8 -*-
"""
Created on Mon Apr  5 17:00:25 2021

@author: Enrique Benavides
"""

import serial
import numpy as np
import time
import matplotlib.pyplot as plt
from aruco_camera import Camera
import cv2

def aruco_pose(frame, aruco_dict, aruco_params):
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

    tag_locations = {}

    if len(corners) > 0:
        ids = ids.flatten()

        for (tag_corners, tag_id) in zip(corners, ids):
            corners = tag_corners.reshape((4,2))
            (top_left, top_right, bottom_right, bottom_left) = corners

            top_right = (int(top_right[0]), int(top_right[1]))
            top_left = (int(top_left[0]), int(top_left[1]))
            bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
            bottom_left = (int(bottom_left[0]), int(bottom_left[1]))

            center_x = int((top_left[0] + bottom_right[0]) / 2.0)
            center_y = int((top_left[1] + bottom_right[1]) / 2.0)
            
            a = bottom_right[0] - bottom_left[0]
            b = bottom_right[1] - bottom_left[1]
            
            center_theta = np.arctan2(b, a)
            
            tag_locations[tag_id] = [(center_x, center_y), center_theta]

        print(tag_locations)

    return tag_locations

def resize(image, width=None, height=None, inter=cv2.INTER_AREA):
    # initialize the dimensions of the image to be resized and
    # grab the image size
    dim = None
    (h, w) = image.shape[:2]

    # if both the width and height are None, then return the
    # original image
    if width is None and height is None:
        return image

    # check to see if the width is None
    if width is None:
        # calculate the ratio of the height and construct the
        # dimensions
        r = height / float(h)
        dim = (int(w * r), height)

    # otherwise, the height is None
    else:
        # calculate the ratio of the width and construct the
        # dimensions
        r = width / float(w)
        dim = (width, int(h * r))

    # resize the image
    resized = cv2.resize(image, dim, interpolation=inter)

    # return the resized image
    return resized

robots = dict(ram01 = "COM9", 
              ram02 = "COM11", 
              ram03 = "COM14", 
              ram04 = "COM16", 
              ram05 = "COM17",
              ram = "COM4")

ARUCO_DICT = {
    	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
    }

refD = []; refI = []; t = []
salD = []; salI = []; velD = []; velI = []
tpose = []

## Variables del robot
r_ll = 0.067            # Llanta del robot 67 mm
L = 0.15                # Distancia entre llantas 150 mm

## Variables de movimiento
V0 = 0.6;
rho0 = 0.3;
w0 = V0/rho0;

vd = (2*V0 + w0*L)/(2*r_ll)
vi = (2*V0 - w0*L)/(2*r_ll)

aruco_key = "DICT_4X4_50"
aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_key])
aruco_params = cv2.aruco.DetectorParameters_create()

print('RAM01')
RAM01 = serial.Serial(robots['ram'],9600)
time.sleep(2)

print("Starting video stream...")
vs = Camera(src = 1).start()
time.sleep(2.0)


data = RAM01.read_until()


#### Lectura base
data = RAM01.read_until()
a2 = data[0:len(data)-2].decode()
valuesfloat = np.array(a2.split(","),dtype=float)
t0 = time.time()


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
        frame = vs.read()
        frame = resize(frame, width = 600)
        
        if (RAM01.inWaiting() > 0):
            data = RAM01.read_until()
            a2 = data[0:len(data)-2].decode()
            valuesfloat = np.array(a2.split(","),dtype=float)
            tag_poses = aruco_pose(frame, aruco_dict, aruco_params)
            print(tag_poses)
            print(valuesfloat)
            salD = np.append(salD,valuesfloat[0])
            salI = np.append(salI,valuesfloat[1])
            velD = np.append(velD,valuesfloat[2])
            velI = np.append(velI,valuesfloat[3])
            refD.append(rd)
            refI.append(ri)
            tact = time.time()
            t.append(tact-t0)
            tpose.append(tag_poses)
            
            
        if (tact-t0 >= 4) and cambio == 0:
            cad = "@W" + str(round(vd,4)) + "D" + str(round(vd,4)) + "I#"
            rd = vd
            ri = vd
            RAM01.write(cad.encode('ascii'))
            cambio = 1
        if (tact-t0 >= 5) and cambio == 1:
            cad = "@W" + str(round(vd,4)) + "D" + str(round(vi,4)) + "I#"
            rd = vd
            ri = vi
            RAM01.write(cad.encode('ascii'))
            cambio = 2
        if (tact-t0 >= 15) and cambio == 2:
            cad = "@W0D0I#"
            rd = 0
            ri = 0
            RAM01.write(cad.encode('ascii'))
            cambio = 3
        if (tact-t0 >= 20) and cambio == 3:
            break
except KeyboardInterrupt:
    pass

RAM01.close()
vs.stop()

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
plt.title('Velocidad Izquierda')
plt.plot(t,refI,'k-')
plt.plot(t,velI, 'r-')
plt.legend(['Ref.Izq','Vel.Izq'],loc = 'upper left')

fig = plt.figure(figsize=(10,15))
plt.grid()
plt.title('Salidas PWM')
plt.plot(t,salD,'k-')
plt.plot(t,salI, 'r-')
plt.legend(['Salida. Der','Salida.Izq'],loc = 'upper left')
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  6 19:29:37 2021

@author: enriq
"""

import serial
import numpy as np
import time
from aruco_camera import Camera
import cv2

#### Diccionarios
robots = dict(ram01 = "COM9", 
              ram02 = "COM11", 
              ram03 = "COM14", 
              ram04 = "COM16", 
              ram05 = "COM17")

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

#### Funciones

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

#### Fin de funciones

##### Comunicacion Bluetooth

try:
    print('RAM01')
    RAM01 = serial.Serial(robots['ram01'],9600)
    time.sleep(1.0)
except:
    print("Error Comunicación RAM01")

try:
    print('RAM02')
    RAM02 = serial.Serial(robots['ram02'],9600)
    time.sleep(1.0)
except:
    print("Error Comunicación RAM02")
    

##### Fin de apertura de puertos Bluetooth

##### Iniciar Webcam
try:
    print("Iniciando video...")
    vs = Camera(src = 1).start()
    time.sleep(1.0)
    # print('camera ready')
except:
    print("Error con video")

#### Iniciar Aruco
aruco_key = "DICT_4X4_1000"
aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_key])
aruco_params = cv2.aruco.DetectorParameters_create()

##### Código

### Variables de robots
r_ll =np.array([0.067,0.067])             # Llanta del robot 67 mm para RAM01 y 02 65mm el resto
L = 0.15                                        # Distancia entre llantas 150 mm

### Variables generales
tpose = []; t = []; cad = []

### Variables de movimiento
V0 = 0.6;                                       # Velocidad lineal m/s
rho0 = 0.3;                                     # Radio de giro 
w0 = V0/rho0;

vd = (2*V0 + w0*L)/(2*r_ll)
vi = (2*V0 - w0*L)/(2*r_ll)

cambio = 0

### Lecturas base
t0 = time.time()
tant = 0
t.append(0)

print('starting main loop')
try:
    while True:
        # frame = vs.read()
        # frame = resize(frame, width = 600)
        tact = time.time() - t0
        
        # if (RAM01.inWaiting() > 0):
            # data = RAM01.read_until()
            # a2 = data[0:len(data)-2].decode()
            # print(a2)
            
        ## Encontrar tag de Aruco
        
        # if (tact - tant) >= 0.05:
        #     tag_poses = aruco_pose(frame, aruco_dict, aruco_params)
        #     t.append(tact)
        #     tpose.append(tag_poses)
        #     print(tag_poses)
            
        ## Enviar información a robots
        
        if (tact>= 4) and cambio == 0:
            
            cad = "@W" + str(np.round(vd[0],4)) + "D" + str(np.round(vd[0],4)) + "I#"
            print(cad)
            RAM01.write(cad.encode('ascii'))
            
            cad = "@W" + str(np.round(vd[1],4)) + "D" + str(np.round(vd[1],4)) + "I#"
            print(cad)
            RAM02.write(cad.encode('ascii'))
            
            cambio = 1
            print('state done 01')
        
        if (tact>= 5) and cambio == 1:
            cad = "@W" + str(np.round(vd[0],4)) + "D" + str(np.round(vi[0],4)) + "I#"
            RAM01.write(cad.encode('ascii'))
            print(cad)
            
            cad = "@W" + str(np.round(vd[1],4)) + "D" + str(np.round(vi[1],4)) + "I#"
            RAM02.write(cad.encode('ascii'))
            print(cad)
            
            cambio = 2
            print('state done 02')
        
        if (tact >= 10) and cambio == 2:
            cad = "@W0D0I#"
            RAM01.write(cad.encode('ascii'))
            RAM02.write(cad.encode('ascii'))
            cambio = 3
            print('state done 03')
        
        if (tact >= 15) and cambio == 3:
            break
        
        ## Mostrar imagen
        # cv2.imshow("Frame", frame)
        
        tant = tact
except KeyboardInterrupt:
    pass

RAM01.close()
RAM02.close()
vs.stop()
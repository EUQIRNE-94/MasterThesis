# -*- coding: utf-8 -*-
"""
Created on Mon Apr 12 2021

@author: Enrique Benavides
"""
## Librerias ##
import serial
import time
from aruco_camera import Camera, ArucoTracker
import cv2
import numpy as np
import matplotlib.pyplot as plt
import funciones as fx
import math

### Diccionarios ###
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
### Diccionarios ###

### Funciones ###

def main():
    print("Iniciando Programa")
    tracker_top = ArucoTracker()
    tracker_bottom = ArucoTracker()
    ##### Variables Fijas #####
    #### Variables de movimiento ####
    V0 = 0.2;                                       # Velocidad lineal m/s
    rho0 = 0.3                                     # Radio de giro 
    w0 = V0/rho0;                                   # Frecuencia natural
    
    #### Partículas en el plano ####
    N = 1;
    M = N;
    R0 = 0 + 0*1j;
    kappa = 0.1*w0;
    A = np.ones([N,N]);
    
    #### Variables del robot ####
    r_llanta = 0.067/2
    L = 0.15
    Ks = np.array([[5,0.1,0.5,1,0.1,0.5]])  # [Cx,Cxk,Cxi,Cy,Cyk,K]
    
    #### Variables de tiempo ####
    tf = 10
    dt = 0.1
    tact = 0; tant = 0
    
    #####
    #### Conexión con Robots ####
    try:
        print('RAM01')
        RAM01 = serial.Serial(robots['ram01'],9600)
        Rob1_ok = True
        time.sleep(1.0)
    except:
        Rob1_ok = False
        print("Error Comunicación RAM01")
    
    try:
        print('RAM02')
        RAM01 = serial.Serial(robots['ram02'],9600)
        Rob2_ok = True
        time.sleep(1.0)
    except:
        Rob2_ok = False
        print("Error Comunicación RAM02")
        
        
    #### Conexión con cámara ####
    camera_top = Camera(src = 2).start()
    camera_bottom = Camera(src = 1).start()
    
    #### Variables para graficar ####
    x = np.zeros((1,N)); y = np.zeros((1,N)); xi = np.zeros((1,N))
    Xr = np.zeros((1,N)); Yr = np.zeros((1,N)); Xir = np.zeros((1,N))
    err_x = np.zeros((1,N)); err_y = np.zeros((1,N)); err_xi = np.zeros((1,N))
    v = np.zeros((1,N)); w = np.zeros((1,N))
    r = np.zeros((1,N),dtype=complex); theta = np.zeros((1,N))
    t = []; t2 = []
    
    #### Condiciones iniciales ####
    cal = False
    while cal == False:
        frame_bottom = camera_bottom.read()
        frame_top = camera_top.read()
        
        frame_top = cv2.cvtColor(frame_top, cv2.COLOR_BGR2GRAY)
        ret3_top,frame_top = cv2.threshold(frame_top,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        
        frame_bottom = cv2.cvtColor(frame_bottom, cv2.COLOR_BGR2GRAY)
        ret3_bottom,frame_bottom = cv2.threshold(frame_bottom,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        tag_poses = tracker_top.aruco_pose(frame_top) # returns dictionary with values 'x', 'y', 'theta'
        tag_poses2 = tracker_bottom.aruco_pose(frame_bottom)

        tag_poses.update(tag_poses2)
        # print(tag_poses)
        try:
            X0 = tag_poses['1']['x']; Y0 = tag_poses['1']['y']; Xi0 = -tag_poses['1']['theta']
            cal = True
        except:
            print("np tag")
        
    #### Prealocación de variables ####
    r0 = X0 + Y0*1j
    theta0 = Xi0 + math.pi/2
    t0 = time.time()
    
    tant_a = 0
    t = np.append(t,tact)
    t2 = np.append(t2,tact)
    
    r_ant = np.zeros((1,N),dtype = complex)
    theta_ant = np.zeros((1,N))
    r_ant[0,:] = r0
    theta_ant[0,:] = theta0
    r[0,:] = r0
    theta[0,:] = theta0
    x[0,:] = X0
    y[0,:] = Y0
    xi[0,:] = Xi0
    Xr[0,:] = X0
    Yr[0,:] = Y0
    Xir[0,:] = Xi0
    kk = 0
    
    while tact <= tf:
        tact = time.time() - t0
        tact_a = tact
        
        if (tact_a - tant_a) >= dt/5:
            
            ## Obtener posición y orientación
            frame_bottom = camera_bottom.read()
            frame_top = camera_top.read()
        
            frame_top = cv2.cvtColor(frame_top, cv2.COLOR_BGR2GRAY)
            ret3_top,frame_top = cv2.threshold(frame_top,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            
            frame_bottom = cv2.cvtColor(frame_bottom, cv2.COLOR_BGR2GRAY)
            ret3_bottom,frame_bottom = cv2.threshold(frame_bottom,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

            tag_poses = tracker_top.aruco_pose(frame_top) # returns dictionary with values 'x', 'y', 'theta'
            tag_poses2 = tracker_bottom.aruco_pose(frame_bottom)
            
            if tracker_top.cs['x'] == 0.0 or tracker_bottom.cs['x'] == 0:
                print('Problema encontrar centro virtual')
                break

            tag_poses.update(tag_poses2)
            
            try:
                x_par = tag_poses['1']['x']; y_par = tag_poses['1']['y']; xi_par = -tag_poses['1']['theta']
                print(tag_poses['1'])
                t = np.append(t,tact_a)
                x = np.append(x,x_par)
                y = np.append(y,y_par)
                xi = np.append(xi,xi_par)
            except:
                print("No ARUCO")
            
            tag_poses.clear()
            tag_poses2.clear()
            tant_a = tact_a
        
        if tact - tant >= dt:
            dtt = tact - tant
            #### Partículas en el plano ####
            # u_par = fx.steeringControl(V0,w0,r_ant,theta_ant,R0,kappa,N,M,A)
            u_par = np.array([w0])
            [r_act,theta_act] = fx.particlesPlane(V0,r_ant,theta_ant,u_par,dtt)
            
            r = np.append(r,r_act)
            theta = np.append(theta,theta_act)
            
            #### Referencias ####
            Xr_par = r_act.real
            Yr_par = r_act.imag
            Xir_par = theta_act
            
            t2 = np.append(t2,tact)
            Xr = np.append(Xr,Xr_par)
            Yr = np.append(Yr,Yr_par)
            Xir = np.append(Xir,Xir_par)
            
            #### Errores ####
            err_xi_par = Xir_par - xi_par
            err_x_par =  np.cos(xi_par)*(Xr_par - x_par) + np.sin(xi_par)*(Yr_par - y_par);
            err_y_par = -np.sin(xi_par)*(Xr_par - x_par) + np.cos(xi_par)*(Yr_par - y_par);
            
            err_xi = np.append(err_xi,err_xi_par)
            err_x = np.append(err_x,err_x_par)
            err_y = np.append(err_y,err_y_par)
            
            #### Control Uniciclo ####
            
            [v_par,w_par] = fx.trackingControl(V0,fx.etheta(theta_act,V0),u_par,err_xi_par,err_x_par,err_y_par,Ks,A)
            
            v = np.append(v,v_par)
            w = np.append(w,w_par)
            
            #### Calcular velocidades de llanta #### Giro estático
            vd = (2*V0 + w0*L)/(2*r_llanta)
            vi = (2*V0 - w0*L)/(2*r_llanta)
            
            #### Calcular velocidades de llanta #### Giro control de posición
            # vd = (2*v_par + w_par*L)/(2*r_llanta)
            # vi = (2*v_par - w_par*L)/(2*r_llanta)
            
            #### Enviar datos a uniciclo ####
            if Rob1_ok == True:
                cad = "@W" + str(round(vd,4)) + "D" + str(round(vi,4)) + "I#"
                print(cad)
                RAM01.write(cad.encode('ascii'))
            
            
            theta_ant = theta_act
            r_ant = r_act
            tant = tact
    
    print(tracker_top.cs)
    print(tracker_bottom.cs)
    cad = "@W0D0I#"
    RAM01.write(cad.encode('ascii'))
    
    if Rob1_ok == True:
        RAM01.close()
    
    camera_top.stop()
    camera_bottom.stop()
    
    
    #### Impresión de variables ####
    plt.close('all')

    fig, axs = plt.subplots(3)
    fig.suptitle('Estados')
    axs[0].grid()
    axs[1].grid()
    axs[2].grid()
    axs[0].plot(t,x,'r-')
    axs[0].plot(t2,Xr,'k-')
    axs[1].plot(t,y, 'r-')
    axs[1].plot(t2,Yr,'k-')
    axs[2].plot(t,xi,'r-')
    axs[2].plot(t2,Xir,'k-')
    
    fig = plt.figure(figsize=(10,15))
    plt.grid()
    plt.title('Plano Fase')
    plt.plot(Xr,Yr,'k-')
    plt.plot(x,y,'r-')
    
### Funciones ###

########## Programa ##########
if __name__ == '__main__':
    main()
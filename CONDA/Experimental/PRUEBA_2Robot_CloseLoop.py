# -*- coding: utf-8 -*-
"""
Created on Mon Apr 12 2021

@author: Enrique Benavides
"""
## Librerias ##
import serial
import time
import sys
from aruco_camera import Camera, ArucoTracker
import cv2
import numpy as np
import matplotlib.pyplot as plt
import funciones as fx

### Diccionarios ###
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
### Diccionarios ###

### Funciones ###

def main():
    print("Iniciando Programa")
    tracker_top = ArucoTracker()
    tracker_bottom = ArucoTracker()
    ##### Variables Fijas #####
    #### Variables de movimiento ####
    V0 = 0.15;                                       # Velocidad lineal m/s
    rho0 = 0.4                                     # Radio de giro 
    w0 = V0/rho0;                                   # Frecuencia natural
    
    #### Partículas en el plano ####
    N = 2;
    M = N;
    R0 = 0 + 0*1j;
    kappa = w0;
    A = np.ones([N,N]);
    
    #### Variables del robot ####
    r_llanta = 0.032/2
    L = 0.085
    # Ks = np.array([[0.005,0.1,0.005,0.5,0.1,0.5]])  # [Cx,Cxk,Cxi,Cy,Cyk,K]
    Ks = np.array([[0.015,0.1,0.005,1.5,0.1,0.5]])  # [Cx,Cxk,Cxi,Cy,Cyk,K]
    Ks = np.array([[0.005,0.1,0.005,0.5,0.1,0.5]])  # [Cx,Cxk,Cxi,Cy,Cyk,K]
    
    #### Variables de tiempo ####
    tf = 30
    dt = 0.1
    tact = 0; tant = 0
    
    #####
    #### Conexión con Robots ####
    tag1 = '1'
    tag2 = '2'
    try:
        print('RAM01')
        RAM01 = serial.Serial(robots['ram01'],9600)
        time.sleep(1.0)
    except:
        sys.exit()
        print("Error Comunicación RAM01")
    
    try:
        print('RAM02')
        RAM02 = serial.Serial(robots['ram03'],9600)
        time.sleep(1.0)
    except:
        sys.exit()
        print("Error Comunicación RAM03")
        
        
    #### Conexión con cámara ####
    camera_top = Camera(src = 2).start()
    camera_bottom = Camera(src = 1).start()
    
    #### Variables para graficar ####
    x_par = np.zeros((1,N)); y_par = np.zeros((1,N)); xi_par = np.zeros((1,N))
    x = np.zeros((1,N)); y = np.zeros((1,N)); xi = np.zeros((1,N))
    
    Xr = np.zeros((1,N)); Yr = np.zeros((1,N)); Xir = np.zeros((1,N))
    err_x = np.zeros((1,N)); err_y = np.zeros((1,N)); err_xi = np.zeros((1,N))
    v = np.zeros((1,N)); w = np.zeros((1,N))
    r = np.zeros((1,N),dtype=complex); theta = np.zeros((1,N))
    t = []; t2 = []
    
    #### Condiciones iniciales ####
    X0 = np.zeros((1,N)); Y0 = np.zeros((1,N)); Xi0 = np.zeros((1,N))
    cal = False;
    adjust_cs = False
    while cal == False:
        frame_bottom = camera_bottom.read()
        frame_top = camera_top.read()
        
        frame_top = cv2.cvtColor(frame_top, cv2.COLOR_BGR2GRAY)
        ret3_top,frame_top = cv2.threshold(frame_top,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        
        frame_bottom = cv2.cvtColor(frame_bottom, cv2.COLOR_BGR2GRAY)
        ret3_bottom,frame_bottom = cv2.threshold(frame_bottom,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        tag_poses = tracker_top.aruco_pose(frame_top) # returns dictionary with values 'x', 'y', 'theta'
        tracker_bottom.calibrated = True
        tracker_bottom.scale = tracker_top.scale
        
        tag_poses2 = tracker_bottom.aruco_pose(frame_bottom)
        if not adjust_cs :
            tracker_bottom.cs['y'] -= 0.04
            tag_poses2 = tracker_bottom.aruco_pose(frame_bottom)
            adjust_cs = True

        tag_poses.update(tag_poses2)
        
        try:
            X0[0,0] = tag_poses[tag1]['x']; Y0[0,0] = tag_poses[tag1]['y']; Xi0[0,0] = -tag_poses[tag1]['theta']
            X0[0,1] = tag_poses[tag2]['x']; Y0[0,1] = tag_poses[tag2]['y']; Xi0[0,1] = -tag_poses[tag2]['theta']
            cal = True
        except:
            print("no tag")
        
    print("5s")
    time.sleep(1)
    print("4s")
    time.sleep(1)
    print("3s")
    time.sleep(1)
    print("2s")
    time.sleep(1)
    print("1s")
    time.sleep(1)
    
    #### Prealocación de variables ####
    r0 = X0 + Y0*1j
    theta0 = Xi0 + np.pi/2
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
    
    x[0,:] = X0; y[0,:] = Y0; xi[0,:] = Xi0
    Xr[0,:] = X0; Yr[0,:] = Y0; Xir[0,:] = Xi0
    
    v_par = np.zeros((1,N))
    w_par = np.zeros((1,N))
    
    print("Iniciando ciclo")
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

            tag_poses.update(tag_poses2)
            
            try:
                x_par[0,0] = tag_poses[tag1]['x']; y_par[0,0] = tag_poses[tag1]['y']; xi_par[0,0] = -tag_poses[tag1]['theta']
                x_par[0,1] = tag_poses[tag2]['x']; y_par[0,1] = tag_poses[tag2]['y']; xi_par[0,1] = -tag_poses[tag2]['theta']
                t = np.append(t,tact_a)
                x = np.append(x,x_par)
                y = np.append(y,y_par)
                xi = np.append(xi,xi_par)
            except:
                # x_par = x_par + v_par*np.cos(xi_par)*(tact_a - tant_a)
                # y_par = y_par + v_par*np.sin(xi_par)*(tact_a - tant_a)
                # xi_par = xi_par + w_par*(tact_a - tant_a)
                print("no tag")
            
            tag_poses.clear()
            tag_poses2.clear()
            tant_a = tact_a
        
        if tact - tant >= dt:
            dtt = tact - tant
            #### Partículas en el plano ####
            u_par = fx.steeringControl(V0,w0,r_ant,theta_ant,R0,kappa,N,M,A)
            # u_par = np.array([w0,w0])
            # u_par = np.array([0,0])
            [r_act,theta_act] = fx.particlesPlane(V0,r_ant,theta_ant,u_par,dtt)
            
            r = np.append(r,r_act)
            theta = np.append(theta,theta_act)
            
            #### Referencias ####
            Xr_par = r_act.real
            Yr_par = r_act.imag
            Xir_par = theta_act
            # print(Xir_par)
            Xir_par = np.mod(Xir_par,(2*np.pi)) - np.pi/2
            for i in range(N):
                if Xir_par[0,i] > np.pi:
                    Xir_par[0,i] -= 2*np.pi
            
            t2 = np.append(t2,tact)
            Xr = np.append(Xr,Xr_par)
            Yr = np.append(Yr,Yr_par)
            Xir = np.append(Xir,Xir_par)
            
            #### Errores ####
            err_xi_par = Xir_par - xi_par
            for i in range(N):
                if np.abs(err_xi_par[0,i]) > np.pi:
                    err_xi_par[0,i] = err_xi_par[0,i] - np.sign(err_xi_par[0,i])*2*np.pi
            
            err_x_par =  np.cos(xi_par)*(Xr_par - x_par) + np.sin(xi_par)*(Yr_par - y_par);
            err_y_par = -np.sin(xi_par)*(Xr_par - x_par) + np.cos(xi_par)*(Yr_par - y_par);
            
            err_xi = np.append(err_xi,err_xi_par)
            err_x = np.append(err_x,err_x_par)
            err_y = np.append(err_y,err_y_par)
            
            #### Control Uniciclo ####
            
            [v_par,w_par] = fx.trackingControl(V0,fx.etheta(theta_act,V0),u_par,err_xi_par,err_x_par,err_y_par,Ks,A)
            
            for i in range(N):
                if v_par[i] < 0:
                    v_par[i] = 0
            
            v = np.append(v,v_par)
            w = np.append(w,w_par)
            
            #### Calcular velocidades de llanta #### Giro estático
            # vd = (2*V0 + w0*L)/(2*r_llanta)
            # vi = (2*V0 - w0*L)/(2*r_llanta)
            
            #### Calcular velocidades de llanta #### Giro control de posición
            vd = (2*v_par + w_par*L)/(2*r_llanta)
            vi = (2*v_par - w_par*L)/(2*r_llanta)
            
            #### Enviar datos a uniciclo ####
            
            cad = "@W" + str(round(vd[0],1)) + "D" + str(round(vi[0],1)) + "I#"
            # print(cad)
            RAM01.write(cad.encode('ascii'))
            cad = "@W" + str(round(vd[1],1)) + "D" + str(round(vi[1],1)) + "I#"
            # print(cad)
            RAM02.write(cad.encode('ascii'))
            
            theta_ant = theta_act
            r_ant = r_act
            tant = tact
            print(tact)
    
    
    cad = "@W0D0I#"
    RAM01.write(cad.encode('ascii'))
    RAM02.write(cad.encode('ascii'))
    
    RAM01.close()
    RAM02.close()
    
    camera_top.stop()
    camera_bottom.stop()
    
    
    #### Impresión de variables ####
    x = x.reshape((-1,N))
    y = y.reshape((-1,N))
    xi = xi.reshape((-1,N))
    
    err_x = err_x.reshape((-1,N))
    err_y = err_y.reshape((-1,N))
    err_xi = err_xi.reshape((-1,N))
    
    v = v.reshape((-1,N))
    w = w.reshape((-1,N))
    
    Xr = Xr.reshape((-1,N))
    Yr = Yr.reshape((-1,N))
    Xir = Xir.reshape((-1,N))
    
    #### Impresión de variables ####
    labels = ["RAM01","RAM02"]
    labels_r = ["RAM01r","RAM02r"]
    color = ['r','g']
    color_r = ['r--','g--']
    plt.close('all')

    fig, axs = plt.subplots(3,figsize=(15,10))
    fig.suptitle('Estados')
    axs[0].grid()
    axs[1].grid()
    axs[2].grid()
    
    for k in range(N):
        axs[0].plot(t,x[:,k], color[k])
        axs[0].plot(t2,Xr[:,k], color_r[k])
        axs[1].plot(t,y[:,k], color[k])
        axs[1].plot(t2,Yr[:,k], color_r[k])
        axs[2].plot(t,xi[:,k], color[k], label = labels[k])
        axs[2].plot(t2,Xir[:,k], color_r[k], label = labels_r[k])
    
    axs[0].set(xlabel = "t [s]", ylabel = "x [m]")
    axs[1].set(xlabel = "t [s]", ylabel = "y [m]")
    axs[2].set(xlabel = "t [s]", ylabel = "xi [rad]")
    axs[2].legend(bbox_to_anchor = (0.5,0), loc = "lower center", bbox_transform = fig.transFigure, ncol = N)
    
    ##### Errores #####
    fig, axs = plt.subplots(3,figsize=(15,10))
    fig.suptitle('Errores')
    axs[0].grid()
    axs[1].grid()
    axs[2].grid()
    
    for k in range(N):
        axs[0].plot(t2,err_x[:,k], color[k])
        axs[1].plot(t2,err_y[:,k], color[k])
        axs[2].plot(t2,err_xi[:,k], color[k], label = labels[k])

    axs[0].set(xlabel = "t [s]", ylabel = "error x [m]")
    axs[1].set(xlabel = "t [s]", ylabel = "error y [m]")
    axs[2].set(xlabel = "t [s]", ylabel = "error xi [rad]")
    axs[2].legend(bbox_to_anchor = (0.5,0), loc = "lower center", bbox_transform = fig.transFigure, ncol = N)
    
    ##### Salidas #####
    fig, axs = plt.subplots(2,figsize = (15,10))
    fig.suptitle('Salidas')
    axs[0].grid()
    axs[1].grid()
    axs[0].set_title("Velocidad lineal")
    axs[1].set_title("Velocidad angular")

    for k in range(N):
        axs[0].plot(t2,v[:,k], color[k])
        axs[1].plot(t2,w[:,k], color[k], label = labels[k])

    axs[0].set(xlabel = "t [s]", ylabel = "[m/s]")
    axs[1].set(xlabel = "t [s]", ylabel = "[rad/s]")
    axs[1].legend(bbox_to_anchor = (0.5,0), loc = "lower center", bbox_transform = fig.transFigure, ncol = N)
    
    ##### Plano Fase #####
    fig = plt.figure(figsize=(10,10))
    plt.grid()
    plt.title('Plano Fase')

    for k in range(N):
        plt.plot(Xr[:,k],Yr[:,k], color[k], label = labels_r[k])
        plt.plot(x[:,k],y[:,k], color_r[k], label = labels[k])

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend(bbox_to_anchor = (0.5,0), loc = "lower center", bbox_transform = fig.transFigure, ncol = N)
    
    np.savez('DATA/data2',robots=N,tiempo=t,tiempo2=t2,Xref=Xr,Yref=Yr,Xiref=Xir,xest=x,yest=y,xiest=xi,errx=err_x,erry=err_y,errxi=err_xi,vellin=v,velang=w) #,veleva=v_eva

########## Programa ##########
if __name__ == '__main__':
    main()
# -*- coding: utf-8 -*-
"""
Created on Mon Feb  1 18:58:00 2021

@author: enriqueBenavides


"""
import numpy as np
from math import pi

def etheta(theta,V0):
    
    y = V0*(np.cos(theta) + np.sin(theta)*1j)
    
    return y

def steeringControl(V0,w0,r,theta,R,kappa,N,M,A):
    
    ## Control espacial ##
    rt = r - R
    rp = etheta(theta,V0)
    dp = np.real(rt)*np.real(rp) + np.imag(rt)*np.imag(rp)
    U1 = w0*(1 + (2/V0)*kappa*dp)
    
    ## Control fase ##
    s_theta = np.sin(theta)
    c_theta = np.cos(theta)
    u2 = np.diag(s_theta.flatten())@A@c_theta.T - np.diag(c_theta.flatten())@A@s_theta.T
    U2 = (kappa/N)*u2.T
    
    ## Control Simétrico ##
    r3 = np.zeros([N,M])
    km = np.ones([M,1])*w0*.1
    km[M-1,0] = -km[M-1,0]
    for m in range(M):
        sm_theta = np.sin((m+1)*theta)
        cm_theta = np.cos((m+1)*theta)
        parcial = (km[m]/(m+1))*(np.diag(sm_theta.flatten())@A@cm_theta.T - np.diag(cm_theta.flatten())@A@sm_theta.T)
        r3[:,m] = parcial.flatten()
    
    U3 = r3.sum(axis=1)
    
    u = U1 + 0*U2 + U3
    
    return u

def particlesPlane(V0,r_in,theta_in,u_in,dt):
    
    r = r_in + etheta(theta_in, V0)*dt
    theta = theta_in + u_in*dt
    
    return [r,theta]

def trackingControl(V0,etheta,dtheta,err_xi,err_x,err_y,Ks,A):
    ## Acomodo de variables
    Ks = Ks.flatten()
    etheta = etheta.flatten()
    dtheta = dtheta.flatten()
    err_xi = err_xi.flatten()
    err_x = err_x.flatten()
    err_y = err_y.flatten()
    ## Ganancias de control
    Cx = Ks[0]; Cxk = Ks[1]; Ctheta = Ks[2]; Cy = Ks[3]; Cyk = Ks[4]; K = Ks[5]
    ## Uniciclos
    i = np.size(etheta)
    ## Prealocacion de variables
    norm_v = np.zeros(i); copX = np.zeros(i); copY = np.zeros(i); k1 = np.zeros(i)
    Kalpha = np.zeros(i)
    
    for k in range(i):
        norm_v[k] = np.linalg.norm(etheta[k])
        
        ## Control de giro
        if err_xi[k] <= 1e-6:
            k1[k] = 1
        else:
            k1 = np.sin(err_xi[k])/err_xi
            
        ## Error de acoplamiento
        for j in range(i):
            copX[k] = copX[k] + A[k,j]*(err_x[k] - err_x[j])
            copY[k] = copY[k] + A[k,j]*(err_y[k] - err_y[j])
            
    v = norm_v*np.cos(err_xi) + Cx*(err_x + Cxk*copX)
    
    for k in range(i):
        if np.abs(v[k]) > 2*V0:
            v[k] = 2*V0
    
    alpha = np.sqrt(K*K + err_x*err_x + err_y*err_y)
    for k in range(i):
        if alpha[k] < 1e-06:
            Kalpha[k] = 1
        else:
            Kalpha[k] = K/alpha[k]
    
    
    w = dtheta + Ctheta*err_xi + ((norm_v*Cy)*(Kalpha)*(np.sin(err_xi)/err_xi))*(err_y + Cyk*copY)
    return [v,w]

def trackingControl2(V0,vr,wr,err_xi,err_x,err_y,Ks,A):
    ## Acomodo de variables
    Ks = Ks.flatten()
    vr = vr.flatten()
    wr = vr.flatten()
    err_xi = err_xi.flatten()
    err_x = err_x.flatten()
    err_y = err_y.flatten()
    ## Ganancias de control
    Cx = Ks[0]; Cxk = Ks[1]; Ctheta = Ks[2]; Cy = Ks[3]; Cyk = Ks[4]; K = Ks[5]
    ## Uniciclos
    i = np.size(err_xi)
    ## Prealocacion de variables
    norm_v = np.zeros(i); copX = np.zeros(i); copY = np.zeros(i); k1 = np.zeros(i)
    Kalpha = np.zeros(i)
    
    for k in range(i):
        
        ## Control de giro
        if err_xi[k] <= 1e-6:
            k1[k] = 1
        else:
            k1 = np.sin(err_xi[k])/(err_xi + 0.0001)
            
        ## Error de acoplamiento
        for j in range(i):
            copX[k] = copX[k] + A[k,j]*(err_x[k] - err_x[j])
            copY[k] = copY[k] + A[k,j]*(err_y[k] - err_y[j])
        
    norm_v = vr
    # v = norm_v + Cx*(err_x + Cxk*copX)
    v = norm_v*np.cos(err_xi) + Cx*(err_x + Cxk*copX)
    
    for k in range(i):
        if np.abs(v[k]) > 2*V0:
            v[k] = 2*V0
    
    alpha = np.sqrt(K*K + err_x*err_x + err_y*err_y)
    
    for k in range(i):
        if alpha[k] < 1e-06:
            Kalpha[k] = 1
        else:
            Kalpha[k] = K/alpha[k]
    
    
    w = wr + Ctheta*err_xi + ((norm_v*Cy)*(Kalpha)*(np.sin(err_xi)/(err_xi+.0000001)))*(err_y + Cyk*copY)
    return [v,w]

def unicycleModel(x_in,y_in,xi_in,v,w,dt):
    
    
    x = x_in + v*np.cos(xi_in)*dt
    y = y_in + v*np.sin(xi_in)*dt
    xi = xi_in + w*dt
    
    return[x,y,xi]

def evasion(x,y,d1,d2):
    # Variables
    N = np.size(x)
    rep = np.zeros([N,N])
    d = np.zeros([N,N])
    
    # Calcular factor de evasion
    for k in range(N):
        for j in range(N):
            d[k,j] = np.linalg.norm([x[k]-x[j],y[k]-y[j]])
            if d[k,j] < d2 and d[k,j] > (d1-0.05):
                rep[k,j] = np.tanh(k*( (d2-d[k,j])/(d2-d1) ))
    
    rep = 1 - rep
    rep = rep*np.tril(np.ones([N,N]),-1) + np.triu(np.ones([N,N]),0)
    rep = rep.min(1)
    
    return rep

########## Matrices de adyacencia y comunicación ##########

def anilloB(N):
    
    A = np.zeros([N,N])
    k = 1
    kk = N-1
    for i in range(N):
        for j in range(N):
            if j == k:
                A[i,j] = 1
            if j == kk:
                A[i,j] = 1
        k = k + 1
        kk = kk + 1
        if k == N:
            k = 0
        if kk == N:
            kk = 0
    
    return A

def anillo(N):
    
    A = np.zeros([N,N])
    k = 1
    for i in range(N):
        for j in range(N):
            if j == k:
                A[i,j] = 1
        k = k + 1
        if k == N:
            k = 0
    
    return A

def matForm(vec,A):
    
    N = np.size(vec)
    
    for k in range(N):
        A[vec[k],:] = 0
        A[:,vec[k]] = 0
    
    return A

########## Formaciones y Simulacion ##########

def unicycleFigure(x,y,theta,escala):
    
    p = escala*np.array([[-1,1,1.5,1,-1],[-1,-1,0,1,1]])
    rot = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    
    trans = np.array([[x],[y]])*np.ones([2,5])
    
    points = rot@p + trans
    
    return points

def lineaForm(V0,w0,r,theta,R,part,d_ang,i):
    r = r.flatten()
    theta = theta.flatten()
    theta = np.mod(theta,2*np.pi)
    
    for kk in range(i):
        if theta[kk] > np.pi:
            theta[kk] -= 2*np.pi
    
    ## Prealocación de variables ##
    d_gamma = np.deg2rad((360/i) - d_ang)
    d_X = np.zeros(i); d_Y = np.zeros(i); d_XI = np.zeros(i)
    rho = V0/w0;
    
    ## Orden de partículas ##
    deltas = theta - theta[part]
    deltas = np.round(deltas/(2*pi/i)) - i
    deltas = np.mod(deltas ,-i)
    # print(deltas)
    
    for k in range(i):
        ## Referencia angular ##
        d_XI[k] = theta[k] - deltas[k]*d_gamma
        if d_XI[k] > np.pi:
            d_XI[k] -= 2*np.pi
        ## Referencia espacial ##
        d_X[k] = rho*np.cos(-pi/2 + d_XI[k])
        d_Y[k] = rho*np.sin(-pi/2 + d_XI[k])
    
    Xr = R.real*np.ones(i) + d_X
    Yr = R.imag*np.ones(i) + d_Y
    XIr = d_XI
    
    return [Xr,Yr,XIr]

def circForm(r,theta,R,part,radio,i):
    r = r.flatten()
    theta = theta.flatten()
    theta = np.mod(theta,2*np.pi)
    
    for kk in range(i):
        if theta[kk] > np.pi:
            theta[kk] -= 2*np.pi
    ### radio del circulo ###
    a = radio
    
    ## Orden de partículas ##
    deltas = theta - theta[part]
    deltas = np.round(deltas/(2*pi/i)) - i
    deltas = np.mod(deltas ,-i)
    # print(deltas)
    
    ## Prealocación de variables ##
    d_X = np.zeros(i); d_Y = np.zeros(i); d_XI = np.zeros(i)
    ang = np.zeros(i)
    
    angulos = np.mod(theta,2*pi)
    for kk in range(i):
        if angulos[kk] > np.pi:
            angulos[kk] -= 2*np.pi
    
    for k in range(i):
        ## Referencia espacial ##
        d_X[k] = r[part].real + a*np.cos(theta[part] + deltas[k]*((2*pi)/i))
        d_Y[k] = r[part].imag + a*np.sin(theta[part] + deltas[k]*((2*pi)/i))
        ## Referencia angular ##
        ang[k] = np.arctan2(d_Y[k]-R.imag,d_X[k]-R.real)
        ang[k] = ang[k] + pi/2
        ang[k] = np.mod(ang[k],2*pi)
        if ang[k] > np.pi:
            ang[k] -= 2*np.pi
        
        d_XI[k] = theta[k] + ang[k] - angulos[k]
        
        if d_XI[k] > np.pi:
            d_XI[k] -= 2*np.pi
        if d_XI[k] < -np.pi:
            d_XI[k] += 2*np.pi
        
        
    Xr = d_X
    Yr = d_Y
    XIr = d_XI
    
    return [Xr,Yr,XIr]

def puntaForm(r,theta,R,part,angulo,radio,i):
    r = r.flatten()
    theta = theta.flatten()
    theta = np.mod(theta,2*np.pi)
    
    for kk in range(i):
        if theta[kk] > np.pi:
            theta[kk] -= 2*np.pi
    ### radio del circulo ###
    a = np.zeros(i)
    beta = np.deg2rad(angulo)
    
    ## Orden de partículas ##
    deltas = theta - theta[part]
    deltas = np.round(deltas/(2*pi/i)) - i
    deltas = np.mod(deltas ,-i)
    # print(deltas)
    
    ## Prealocación de variables ##
    d_X = np.zeros(i); d_Y = np.zeros(i); d_XI = np.zeros(i)
    ang = np.zeros(i)
    
    angulos = np.mod(theta,2*pi)
    
    for k in range(i):
        pp = np.ceil(np.abs(deltas[k])/2)
        a[k] = radio*pp
    
    for k in range(i):
        ## Referencia espacial ##
        d_X[k] = r[part].real + a[k]*np.cos(theta[part] + ((-1)**np.abs(deltas[k]))*beta)
        d_Y[k] = r[part].imag + a[k]*np.sin(theta[part] + ((-1)**np.abs(deltas[k]))*beta)
        ## Referencia Angular ##
        ang[k] = np.arctan2(d_Y[k]-R.imag,d_X[k]-R.real)
            
        ang[k] = ang[k] + pi/2
        if ang[k] > np.pi:
            ang[k] -= 2*pi
        if d_XI[k] < -np.pi:
            d_XI[k] += 2*np.pi
        
        d_XI[k] = theta[k] + ang[k] - angulos[k]
        
        if d_XI[k] > np.pi:
            d_XI[k] -= 2*np.pi
        if d_XI[k] < -np.pi:
            d_XI[k] += 2*np.pi
        
        d_XI[part] = theta[part]
        
    Xr = d_X
    Yr = d_Y
    XIr = d_XI
    
    return [Xr,Yr,XIr]

def subLineaForm(V0,w0,theta,R,SubG,d_ang,i):
    theta = theta.flatten()
    theta = np.mod(theta,2*np.pi)
    
    for kk in range(i):
        if theta[kk] > np.pi:
            theta[kk] -= 2*np.pi
    ## Prealocacion de variables
    d_gamma = np.deg2rad((360/i) - d_ang)
    rho = V0/w0
    
    ## Prealocación de variables ##
    d_X = np.zeros(i); d_Y = np.zeros(i); d_XI = np.zeros(i)
    
    if SubG >= 2 and SubG < i:
        particulas = np.zeros(SubG)
        Lid = np.zeros(SubG)
        
        kk = 0
        for k in range(i):
            particulas[kk] = particulas[kk] + 1
            kk = kk + 1
            if kk == SubG:
                kk = 0
        
        for k in range(1,SubG):
            Lid[k] = Lid[k-1] + particulas[k-1]
            
        ## Orden de partículas ##
        deltas = theta - theta[0]
        deltas = np.round(deltas/(2*pi/i)) - i
        deltas = np.mod(deltas ,-i)
        
        for j in range(SubG-1):
            for k in range(i):
                if np.abs(deltas[k]) >= (Lid[j] + particulas[j]) and np.abs(deltas[k]) < (Lid[j+1] + particulas[j+1]):
                    deltas[k] = deltas[k] + Lid[j+1]
        # print(deltas)
    else:
        ## Orden de partículas ##
        deltas = theta - theta[1]
        deltas = np.round(deltas/(2*pi/i)) - i
        deltas = np.mod(deltas ,-i)
        # print(deltas)
        
    for k in range(i):
        ## Referencia angular ##
        d_XI[k] = theta[k] - deltas[k]*d_gamma
        if d_XI[k] > np.pi:
            d_XI[k] -= 2*np.pi
        ## Referencia espacial ##
        d_X[k] = rho*np.cos(-pi/2 + d_XI[k])
        d_Y[k] = rho*np.sin(-pi/2 + d_XI[k])
    
    Xr = R.real*np.ones(i) + d_X
    Yr = R.imag*np.ones(i) + d_Y
    XIr = d_XI
        
    return [Xr,Yr,XIr]

def subCircForm(r,theta,R,SubG,radio,i):
    r = r.flatten()
    theta = theta.flatten()
    theta = np.mod(theta,2*np.pi)
    
    for kk in range(i):
        if theta[kk] > np.pi:
            theta[kk] -= 2*np.pi
    ### radio del circulo ###
    a = radio
    
    ## Prealocación de variables ##
    d_X = np.zeros(i); d_Y = np.zeros(i); d_XI = np.zeros(i)
    ang = np.zeros(i); lids = np.zeros(i); angs = np.ones(i)
    
    if SubG >= 2 and SubG < i:
        particulas = np.zeros(SubG)
        Lid = np.zeros(SubG)
        
        kk = 0
        for k in range(i):
            particulas[kk] = particulas[kk] + 1
            kk = kk + 1
            if kk == SubG:
                kk = 0
        
        for k in range(1,SubG):
            Lid[k] = Lid[k-1] + particulas[k-1]
            
        angs = angs*((2*pi)/particulas[0])
        
        ## Orden de partículas ##
        deltas = theta - theta[0]
        deltas = np.round(deltas/(2*pi/i)) - i
        deltas = np.mod(deltas ,-i)
        
        for j in range(SubG-1):
            index = np.where(np.abs(deltas) == Lid[j+1])
            for k in range(i):
                if np.abs(deltas[k]) >= (Lid[j] + particulas[j]) and np.abs(deltas[k]) < (Lid[j+1] + particulas[j+1]):
                    deltas[k] = deltas[k] + Lid[j+1]
                    lids[k] = index[0][0];
                    angs[k] = (2*pi)/particulas[j+1]

    else:
        ## Orden de partículas ##
        deltas = theta - theta[1]
        deltas = np.round(deltas/(2*pi/i)) - i
        deltas = np.mod(deltas ,-i)
    
    angulos = np.mod(theta,2*pi)
    
    for k in range(i):
        ## Referencia espacial ##
        part = int(lids[k])
        d_X[k] = r[part].real + a*np.cos(theta[part] + deltas[k]*angs[k])
        d_Y[k] = r[part].imag + a*np.sin(theta[part] + deltas[k]*angs[k])
        
        ## Referencia angular ##
        ang[k] = np.arctan2(d_Y[k]-R.imag,d_X[k]-R.real)
        
        if ang[k] > np.pi:
            ang[k] -= 2*pi
        if ang[k] < -np.pi:
            ang[k] += 2*pi
        
        ang[k] = ang[k] + pi/2
        ang[k] = np.mod(ang[k],2*pi)
        d_XI[k] = theta[k] + ang[k] - angulos[k]
        
        if d_XI[k] > np.pi:
            d_XI[k] -= 2*np.pi
        if d_XI[k] < -np.pi:
            d_XI[k] += 2*np.pi
        
    Xr = d_X
    Yr = d_Y
    XIr = d_XI
        
    return [Xr,Yr,XIr]
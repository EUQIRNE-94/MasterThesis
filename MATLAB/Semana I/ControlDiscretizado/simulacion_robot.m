clear
clc
close all
%%Variables de referencia
xr = 0;         %Punto de inicio en X
yr = 0;         %Punto de inicio en Y
thetar = pi;    %Orientacion de inicio en rad
A = 5;          %Amplitud de la vuelta
f = .01;        %Frecuencia del giro
dt = .01;       %Diferencial del tiempo
t = 0;          %Tiempo de inicio
tfin = 1000;     %Tiempo de fin del ciclo

%Variables de inicio en el bucle de movimiento
theta = thetar;
x = xr;
y = yr;

%Constantes de los errores
kx = .5;
ky = 150;
ktheta = 1;
k = 1;

%%Movimiento del robot
i = 1;

while (t < tfin)
    %%Variables de referencia
    xr = 8;%A*cos(2*pi*f*t);
    yr = 5;%A*sin(2*pi*f*t);
    %Primera derivada
    dx = 2.20;%-2*A*pi*f*sin(2*pi*f*t);
    dy = 2.20;%2*A*pi*f*cos(2*pi*f*t);
    %Segunda derivada
    dxx = .50;%- ((2*pi*f)^2)*A*cos(2*pi*f*t);
    dyy = .50;%- ((2*pi*f)^2)*A*sin(2*pi*f*t);
    
    thetar = pi;
    
    %%Error con respecto al punto anterior
    errX = cos(theta)*(xr - x) + sin(theta)*(yr - y);
    errY = -sin(theta)*(xr - x) + cos(theta)*(yr - y);
    errTheta = thetar - theta;
    
    %%Velocidades referencia
    vr = sqrt((dx^2) + (dy^2));
    wr = (dyy*dx - dxx*dy)/((dx^2) + (dy^2));
    
    %%Velocidades
    alpha = sqrt(k^2 + (errX^2) + (errY^2));
    v = vr*cos(errTheta) + kx*errX;
    if (errTheta == 0)
        w = wr + ktheta*errTheta + vr*(k/alpha)*ky*errY;
    else
        w = wr + ktheta*errTheta + vr*(k/alpha)*(sin(errTheta)/errTheta)*ky*errY;
    end
    
    
    if abs(w) > 1e-6
        a = v/(.5*w)*sin(.5*w*dt);
    else
        a= v*dt;
    end
    
    %%Puntos
    x = x + a*cos(theta + 2*w*.5*dt);
    y = y + a*sin(theta + 2*w*.5*dt);
    theta = theta + w*dt;
    
    
    %Vectores para obtener las graficas del modelo
    p(i,1) = x;
    p(i,2) = y;
    p(i,3) = theta;
    
    vel(i,1) = v;
    vel(i,2) = w;
    tff(i) = t;
    
    p1(i,1) = xr;
    p1(i,2) = yr;
    
    err(i,1) = errX;
    err(i,2) = errY;
    err(i,3) = errTheta;
    
    t = t + dt;
    i = i + 1;
end

figure(1)
plot(p(:,1),p(:,2));
title('Posiciones','Fontsize',14)
xlabel('X (m)','Fontsize',14)
ylabel('Y (m)','Fontsize',14)
hold on
plot(p1(:,1),p1(:,2), 'r--', 'LineWidth', 2);
axis equal
grid on

figure(2)
subplot(2,1,1)
title('Velocidades','Fontsize',14)
plot(tff,vel(:,1));
xlabel('Tiempo','Fontsize',14)
ylabel('Velocidad lineal (m/s)','Fontsize',14)
subplot(2,1,2)
plot(tff,vel(:,2));
xlabel('Tiempo','Fontsize',14)
ylabel('$\dot{\theta}$ (rad)','Interpreter','Latex','Fontsize',14,'Color','k')

figure(3)
subplot(3,1,1)
plot(tff,p(:,1));
xlabel('Tiempo','Fontsize',14)
ylabel('Posicion en X (m)','Fontsize',14)
subplot(3,1,2)
plot(tff,p(:,2));
xlabel('Tiempo','Fontsize',14)
ylabel('Posicion en Y (m)','Fontsize',14)
subplot(3,1,3)
plot(tff,p(:,3));
xlabel('Tiempo','Fontsize',14)
ylabel('$\theta$ (rad)','Interpreter','Latex','Fontsize',14,'Color','k')

figure(4)
subplot(3,1,1)
plot(tff,err(:,1));
xlabel('Tiempo','Fontsize',14)
ylabel('Error X','Fontsize',14)
subplot(3,1,2)
plot(tff,err(:,2));
xlabel('Tiempo','Fontsize',14)
ylabel('Error Y','Fontsize',14)
subplot(3,1,3)
plot(tff,err(:,3));
xlabel('Tiempo','Fontsize',14)
ylabel('Error en $\theta$','Interpreter','Latex','Fontsize',14,'Color','k')
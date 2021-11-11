function [] = Rbotcontrol2()
clear all
close all
clc

global variables;

x0 = [5;0;0];
time = [0 5];
[t,x] = ode113(@botModel,time,x0);

variables.posx = x(:,1);
variables.posy = x(:,2);
variables.posOmega = x(:,3);
variables.time = t;

plotg();
end

function [] = plotg()
global variables;

figure(1)
plot(variables.posx, variables.posy,'b','LineWidth',2);
title('Position','Interpreter','Latex','Fontsize', 14, 'Color','k');
xlabel('X','Interpreter','Latex','Fontsize',16,'Color','k');
ylabel('Y','Interpreter','Latex','Fontsize',16,'Color','k');
grid on

figure(2)
subplot(2,1,1)
plot(variables.time, variables.posx, 'k:','LineWidth', 2);
title('Velocidad de control','Interpreter','Latex','Fontsize',14)
xlabel('Tiempo (s)','Interpreter','Latex','Fontsize',16,'Color','k');
ylabel('Posixion X (m)','Interpreter','Latex','Fontsize',16,'Color','k');
grid on

subplot(2,1,2)
plot(variables.time, variables.posy, 'k:','LineWidth', 2);
title('Velocidad de control','Interpreter','Latex','Fontsize',14)
xlabel('Tiempo (s)','Interpreter','Latex','Fontsize',16,'Color','k');
ylabel('Posixion Y (m)','Interpreter','Latex','Fontsize',16,'Color','k');
grid on

figure(3)
plot(variables.time, variables.posOmega, 'k:','LineWidth', 2);
title('Velocidad de control','Interpreter','Latex','Fontsize',14)
xlabel('Tiempo (s)','Interpreter','Latex','Fontsize',16,'Color','k');
ylabel('Posixion $\theta$ (rad)','Interpreter','Latex','Fontsize',16,'Color','k');
grid on
end

function dx = botModel(t,x)

%%Referencias
vr = 2;
wr = 1;

xr = 5*cos(2*pi*.5*t + pi/2);
yr = 5*sin(2*pi*.5*t + pi/2);
thetar = atan(yr/xr);

%%Constantes proporcion
kx = 10;
ky = 100;
ktheta = 1;
k = 1;

%%Errores
xe = cos(x(3))*(xr - x(1)) + sin(x(3))*(yr - x(2));
ye = -sin(x(3))*(xr-x(1)) + cos(x(3))*(yr - x(2));
thetae = thetar - x(3);



alpha = sqrt((xe^2) + (ye^2));

%%Control de velocidad
v = vr*cos(thetae) + kx*xe;
w = wr + ktheta*thetae + vr*(k/alpha)*ky*ye;

%%Ecuaciones
dx(1) = v*cos(x(3));
dx(2) = v*sin(x(3));
dx(3) = w;
dx = dx';
end
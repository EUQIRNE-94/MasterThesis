function [] = Rbotcontrol()
clear all
close all
clc

global variables;

x0 = [.1;.1;-pi/4];
time = [0 1];
[t,x] = ode45(@botModel,time,x0);

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
ylabel('Y','Interpreter','Latex','Fontsize',16,'Color','k')
grid on

figure(2)
plot(variables.time, variables.posy,'b',variables.time, variables.posx,'r',variables.time, variables.posOmega,'g')
end

function dx = botModel(~,x)
%Velocidad de referencia
vr = 1;
wr = 2;

%Posicion de referencia
xr = 1;
yr = 2;
thetar = pi/2;

%Proporcionalidades de control
kx = .5;
ky = 1;
ktheta = .05;

%Errores
xe = cos(x(3))*(xr - x(1)) + sin(x(3))*(yr - x(2));
ye = -sin(x(3))*(xr - x(1)) + cos(x(3))*(yr - x(2));
thetaerr = thetar - x(3);

%Control de velocidades
v = vr*cos(thetaerr) + kx*xe;
w = wr + ktheta*thetaerr + vr*(sin(thetaerr)/thetaerr)*ye*3;

%Ecuaciones de estado
dx(1) = w*ye - v +vr*cos(thetaerr);
dx(2) = -w*xe + vr*sin(thetaerr);
dx(3) = wr - w;
dx = dx';
end
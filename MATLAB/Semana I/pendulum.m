function [] = pendulum()
clear all
close all
clc

global variables;

x0 = [.1;0];
time = [0 15];
[t,x] = ode45(@pendulumModel,time,x0);

variables.pos = x(:,1);
variables.vel = x(:,2);
variables.time = t;

plotg();
end

function [] = plotg()
global variables;

figure(1)
plot(variables.time, variables.pos,'b','LineWidth',2);
title('Angular Position','Interpreter','Latex','Fontsize', 14, 'Color','k');
xlabel('Time(s)','Interpreter','Latex','Fontsize',16,'Color','k');
ylabel('$\theta$ (rad)','Interpreter','Latex','Fontsize',16,'Color','k')
grid on

figure(2)
plot(variables.time, variables.vel,'r','LineWidth',2);
title('Angular Velocity','Interpreter','Latex','Fontsize', 14, 'Color','k');
xlabel('Time(s)','Interpreter','Latex','Fontsize',16,'Color','k');
ylabel('$\dot{\theta}$ (rad)','Interpreter','Latex','Fontsize',16,'Color','k')
grid on
end

function dx = pendulumModel(~,x)
%Parametros del pendulo
l = .5; %Largo del pendulo (m)
m = 1; %Masa de la bola (kg)
g = 9.81; %Gravedad (m/s^2)
k = 1; %Coeficiente de friccion

d = 1;
k1 = 1;
k2 = .1;


%T = 0; %Torque o entrada de control
T =  -k1*(x(1)-d) -k2*x(2) + (g/l)*sin(d)/(1/(m*l^2)); %Torque o entrada de control (N*m)

dx(1) = x(2);
dx(2) = -(g/l)*sin(x(1)) - (k/m)*x(2) + (1/(m*l^2))*T;
dx = dx';
end
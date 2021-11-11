function [] = Rbot()
clear all
close all
clc

global variables;

x0 = [.1;.1;.1];
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
title('Angular Position','Interpreter','Latex','Fontsize', 14, 'Color','k');
xlabel('X','Interpreter','Latex','Fontsize',16,'Color','k');
ylabel('Y','Interpreter','Latex','Fontsize',16,'Color','k')
grid on

figure(2)
plot(variables.time, variables.posy,'b',variables.time, variables.posx,'r',variables.time, variables.posOmega,'g')
end

function dx = botModel(~,x)
v = 1;
w = 2;

dx(1) = v*cos(x(3));
dx(2) = v*sin(x(3));
dx(3) = w;
dx = dx';
end

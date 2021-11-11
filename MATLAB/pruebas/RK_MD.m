%% Runge-Kutta para multiples dinámicas
% Solucionar dinámica de N partículas oscilantes acopladas
clear
clc
close all

y = rk(@(x)fxy(x),[1,2,3,4,5,6],0,.5);

function y = fxy(x)
y = -2*x^3 + 12*x^2 - 20*x + 8.5;
end

function y = rk(f,y0,i,dt)
% 1 iteracion de Runge-Kutta
k1 = f(i*dt)
k2 = f(i*dt + .5*dt)
k3 = f(i*dt + .5*dt)
k4 = f(i*dt + dt)
y = y0 + (1/6)*(k1 + 2*k2 + 2*k3 + k4)*dt;
end
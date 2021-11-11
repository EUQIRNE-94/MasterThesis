%% Partículas con Dinámica de Oscilador Acoplado
% Pruebas de integrador

% Limpieza de variables alocadas previamente
clear
close all
% clc

global uk

% Condiciones iniciales
rk_0 = 2 + 2i;
theta_0 = 0;
w0 = 1;

% Tiempo
dt = 0.01;
tspan = 0:dt:(2*pi/w0);

% Prealocación de variables
n = length(tspan);
r_k = zeros(n,1);
theta = zeros(n,1);


% Condiciones iniciales
r_k(1,:) = rk_0;
theta(1,:) = theta_0;
uk = w0;

% Integración Runge-Kutta 4 orden
for i = 1:n-1
  r_k(i+1,:) = runk(@(r,theta)fr(r,theta),theta(i,:),r_k(i,:),r_k(i,:),dt);
  theta(i+1,:) = runk(@(r,uk)ftheta(r,uk),theta(i,:),r_k(i,:),theta(i,:),dt);
end

% Prealocacion
ukk = 1;
rkk = zeros(n,1);
thetakk = zeros(n,1);

% Condiciones iniciales
rkk(1) = rk_0;
thetakk(1) = theta_0;
for k = 1:n-1
  rkk (k+1,1) = rkk(k,1) + e_theta(thetakk(k,1))*dt;
  thetakk(k+1,1) = thetakk(k,1) + ukk*dt;
end

% ODE Matlab
% u_k = w0;
% c_ini = [(2 + 2*1i) 0];
% [t,y_t] = ode45(@(t,x)OsAc(t,x,u_k),tspan,c_ini);

figure(1)
plot(real(r_k),imag(r_k))
axis equal
grid on
hold on
plot(real(rkk),imag(rkk))
% plot(real(y_t(:,1)),imag(y_t(:,1)))

figure(2)
plot(tspan,real(r_k-rkk)); hold on
plot(tspan,imag(r_k-rkk))

%% Funcion del modelo
% Ecuacion 1a paper [0]
function dr = fr(theta,~)
dr = cos(theta) + 1i*sin(theta);
end
  
% Ecuacion 1b paper [0]
function dtheta = ftheta(~,~)
global uk
dtheta = uk;
end

function y = e_theta(theta)
y = cos(theta) + 1i*sin(theta);
end

function dx = OsAc(t,x,u_k)
  dx(1) = cos(x(2)) + 1i*sin(x(2));
  dx(2) = u_k;  
  dx = dx';
end

% Integrador Runge-Kutta 4 orden
function y = runk(f,xk,yk,y0,dt)
% 1 iteracion de Runge-Kutta
k1 = f(xk,yk);
k2 = f(xk + .5*dt, yk + .5*k1*dt);
k3 = f(xk + .5*dt, yk + .5*k2*dt);
k4 = f(xk + dt, yk + k3*dt);
y = y0 + (1/6)*(k1 + 2*k2 + 2*k3 + k4)*dt;
end
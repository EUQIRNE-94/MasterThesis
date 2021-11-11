%% Prueba de control de seguimiento a Sepulchre
% 1 Partícula, 1 Uniciclo
% Ecuación (8) Sepulchre [0-02]

clear
clc
close all

%% Variables Fijas
w0 = 1;
rev = 5;
dt = .1;
tspan = 0:dt:rev*(2*pi/w0);
n = length(tspan);
K = -5;

%% Sistema
%   Prealocacion de variables
% Sepulchre
r = zeros(n,1); theta = zeros(n,1); u = zeros(n,1);
% Carrito
x = zeros(n,1); y = zeros(n,1); eta = zeros(n,1);
v = zeros(n-1,1); w = zeros(n-1,1);
err_theta = zeros(n-1,1); err_x = zeros(n-1,1); err_y = zeros(n-1,1);

% Condiciones iniciales Sepulchre
r0 = 2*rand() + 2i*rand();
theta0 = 2*pi*rand();

% Condiciones iniciales carrito
x0 = real(r0) + (.5*rand() - .25);
y0 = imag(r0) + (.5*rand() - .25);
eta0 = theta0 + (.5*rand() - .25);

% Integración del sistema
r(1) = r0;
theta(1) = theta0;
x(1) = x0;
y(1) = y0;
eta(1) = eta0;

for i = 1:n-1
  % Sistema de Sepulchre
  r(i+1) = r(i) + etheta(theta(i))*dt;
  u(i) = u_k(w0,theta(i),1,K);
  theta(i+1) = theta(i) + u(i)*dt;
  
  % Seguimiento del carrito
  err_theta(i+1) = theta(i) - eta(i);
  err_x(i+1) = cos(eta(i))*(real(r(i)) - x(i)) + sin(eta(i))*(imag(r(i)) - y(i));
  err_y(i+1) = -sin(eta(i))*(real(r(i)) - x(i)) + cos(eta(i))*(imag(r(i)) - y(i));
  
  [v(i+1),w(i+1)] = u_k1(etheta(theta(i)),u(i),err_theta(i),err_x(i),err_y(i));
  x(i+1) = x(i) + v(i)*cos(eta(i))*dt;
  y(i+1) = y(i) + v(i)*sin(eta(i))*dt;
  eta(i+1) = eta(i) + w(i)*dt;
  
end

%% Gráficos
% Movimiento en el plano
figure(1)
hold on; grid on; axis equal
plot(r,'linewidth',2)
plot(x,y)


% Errores
figure(2)
subplot(3,1,1)
title('Errores')
hold on; grid on
plot(tspan,err_x)
xlabel('Tiempo [t]')
ylabel('Error X')

subplot(3,1,2)
hold on; grid on
plot(tspan,err_y)
xlabel('Tiempo [t]')
ylabel('Error Y')

subplot(3,1,3)
hold on; grid on
plot(tspan,err_theta)
xlabel('Tiempo [t]')
ylabel('Error \theta')

% Entradas de control
figure(3)
subplot(3,1,1)
hold on; grid on
title('Entrada de control')
plot(tspan,u)
xlabel('Tiempo [t]')
ylabel('u')

subplot(3,1,2)
hold on; grid on
title('Velocidad lineal')
plot(tspan,v)
xlabel('Tiempo [t]')
ylabel('v')

subplot(3,1,3)
hold on; grid on
title('Velocidad angular')
plot(tspan,w)
xlabel('Tiempo [t]')
ylabel('\omega')

%% Funciones
% Sepulchre
function y = etheta(theta)
y = cos(theta) + 1i*sin(theta);
end

% Control
function u = u_k(w0,theta,N,K)
s_theta = sin(theta);
c_theta = cos(theta);

U1 = sum(c_theta)*eye(N)*s_theta' - sum(s_theta)*eye(N)*c_theta';

u = w0 + (K/N)*U1';
end

% Morales//Nijmeier//Gutierrez
function [v,w] = u_k1(etheta,theta_d,err_theta,err_x,err_y)
Cx = 0.5;
Ctheta = 0.5;
Cy = 0.2;
K = 0.5;

v = norm(etheta)*cos(err_theta) + Cx*err_x;

if err_theta <= 1e-6
  k1 = 1;
else
  k1 = sin(err_theta)/err_theta;
end
alpha = sqrt(K^2 + err_x^2 + err_y^2);

w = theta_d + Ctheta*err_theta + norm(etheta)*(k1)*(K/alpha)*Cy*err_y;

end
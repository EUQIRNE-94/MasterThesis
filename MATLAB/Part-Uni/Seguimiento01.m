%% Prueba de control de seguimiento a Sepulchre
% N part?culas con N Uniciclos
% Ecuaci?n (8) Sepulchre [0-02]

clear
clc
close all

%% Variables Fijas
w0 = 1;
rev = 2;
N = 4;
dt = 0.05;
tspan = 0:dt:rev*(2*pi/w0);
n = length(tspan);
K = -w0;

%% Sistema
%   Prealocacion de variables
% Sepulchre
r = zeros(n,N); theta = zeros(n,N); u = zeros(n-1,N);
% Uniciclo
x = zeros(n,N); y = zeros(n,N); eta = zeros(n,N);
v = zeros(n-1,N); w = zeros(n-1,N);
err_theta = zeros(n-1,N); err_x = zeros(n-1,N); err_y = zeros(n-1,N);

% Condiciones iniciales sistema Sepulchre
r0 = 5*rand(1,N) + 5i*rand(1,N);
theta0 = 2*pi*rand(1,N);

% Condiciones iniciales de uniciclos
x0 = real(r0) + (.5*rand(1,N) - .25);
y0 = imag(r0) + (.5*rand(1,N) - .25);
eta0 = theta0 + (.5*rand(1,N) - .25);

% Integraci?n del sistema
r(1,:) = r0;
theta(1,:) = theta0;
x(1,:) = x0;
y(1,:) = y0;
eta(1,:) = eta0;

for i = 1:n-1
  % Sistema de Sepulchre
  r(i+1,:) = r(i,:) + etheta(theta(i,:))*dt;
  u(i,:) = u_k(w0,theta(i,:),1,K);
  
  if i >= n/2
    u(i,:) = u_k(w0,theta(i,:),1,K) - w0;
  else
    u(i,:) = u_k(w0,theta(i,:),1,K);
  end
  
  theta(i+1,:) = theta(i,:) + u(i,:)*dt;
  
  % Seguimiento del carrito
  err_theta(i,:) = theta(i,:) - eta(i,:);
  err_x(i,:) =  cos(eta(i,:)).*(real(r(i,:)) - x(i,:)) + sin(eta(i,:)).*(imag(r(i,:)) - y(i,:));
  err_y(i,:) = -sin(eta(i,:)).*(real(r(i,:)) - x(i,:)) + cos(eta(i,:)).*(imag(r(i,:)) - y(i,:));
  
  [v(i,:),w(i,:)] = u_k1(etheta(theta(i,:)),u(i,:),err_theta(i,:),err_x(i,:),err_y(i,:));
  x(i+1,:) = x(i,:) + v(i,:).*cos(eta(i,:))*dt;
  y(i+1,:) = y(i,:) + v(i,:).*sin(eta(i,:))*dt;
  eta(i+1,:) = eta(i,:) + w(i,:)*dt;
  
end

%% Gr?ficos
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
plot(tspan(1:n-1),err_x)
xlabel('Tiempo [t]')
ylabel('Error X')

subplot(3,1,2)
hold on; grid on
plot(tspan(1:n-1),err_y)
xlabel('Tiempo [t]')
ylabel('Error Y')

subplot(3,1,3)
hold on; grid on
plot(tspan(1:n-1),err_theta)
xlabel('Tiempo [t]')
ylabel('Error \theta')

% Entradas de control
figure(3)
subplot(3,1,1)
hold on; grid on
title('Entrada de control')
plot(tspan(1:n-1),u)
xlabel('Tiempo [t]')
ylabel('u')

subplot(3,1,2)
hold on; grid on
title('Velocidad lineal')
plot(tspan(1:n-1),v)
xlabel('Tiempo [t]')
ylabel('v')

subplot(3,1,3)
hold on; grid on
title('Velocidad angular')
plot(tspan(1:n-1),w)
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
Cxk = 0.1;
Ctheta = 0.5;
Cy = 0.5;
Cyk = 0.1;
K = 0.5;

norma = zeros(1,length(etheta));
copX = zeros(1,length(etheta));
copY = zeros(1,length(etheta));
k1 = zeros(1,length(etheta));

for k = 1:length(etheta)
  norma(1,k) = norm(etheta(k));
  if err_theta(k) <= 1e-6
    k1(1,k) = 1;
  else
    k1(1,k) = sin(err_theta(k))./err_theta(k);
  end
  for j = 1:length(etheta)
    copX(1,k) = copX(1,k) + (err_x(k) - err_x(j));
    copY(1,k) = copY(1,k) + (err_y(k) - err_y(j));
  end
end

v = norma.*cos(err_theta) + Cx*(err_x + Cxk*copX);

alpha = sqrt(K^2 + err_x.^2 + err_y.^2);
w = theta_d + Ctheta.*err_theta + (norma.*(k1)).*(K./alpha).*Cy.*(err_y + Cyk*copY);

end
%% Prueba de control de seguimiento a Sepulchre
% 1 partícula con 2 Uniciclos
% Ecuación (8) Sepulchre [0-02]

clear
clc
close all

%% Variables Fijas
w0 = 1;
rev = 5;
N = 1;      % Particula Sepulchre
dt = 0.05;
tspan = 0:dt:rev*(2*pi/w0);
n = length(tspan);
K = -5;
i = 2;      % Uniciclos

%% Sistema
%   Prealocacion de variables
% Sepulchre
r = zeros(n,N); theta = zeros(n,N); u = zeros(n-1,N); c = zeros(n-1,N);
% Uniciclo
x = zeros(n,i); y = zeros(n,i); eta = zeros(n,i); Xr = zeros(n,i); Yr = zeros(n,i);
v = zeros(n-1,i); w = zeros(n-1,i);
err_theta = zeros(n-1,i); err_x = zeros(n-1,i); err_y = zeros(n-1,i);

% Condiciones iniciales sistema Sepulchre
% r0 = 5*rand() + 5i*rand();
% theta0 = 2*pi*rand();
r0 = 0 - 1i;
theta0 = 0;

% Condiciones iniciales de uniciclos
x0 = real(r0)  + (0.1*rand(1,i) - .05);
y0 = imag(r0)  + (0.1*rand(1,i) - .05);
eta0 = theta0 + (.1*rand(1,i) - .05);

% Integración del sistema
r(1) = r0;
theta(1) = theta0;
c(1,:) = r0 + (1i/w0)*etheta(theta0);

x(1,:) = x0;
y(1,:) = y0;
eta(1,:) = eta0;


for i = 1:n-1
  % Sistema de Sepulchre
  r(i+1) = r(i) + etheta(theta(i))*dt;
  
%   u(i) = u_k(w0,theta(i),1,K);
  if i >= n/2
    u(i,:) = u_k(w0,theta(i,:),1,K) - w0;
  else
    u(i,:) = u_k(w0,theta(i,:),1,K);
  end
  
  theta(i+1) = theta(i) + u(i)*dt;
  c(i,:) = r(i) + (1i/w0)*etheta(theta(i));
  
  % Seguimiento del carrito
  deltatheta = [0,0];
  thetar = theta(i) + deltatheta;
  err_theta(i,:) = thetar - eta(i,:);
  
  Xr(i,:) = real(r(i)) + [0, .5*cos(theta(i)-pi/2)];
  Yr(i,:) = imag(r(i)) + [0, .5*sin(theta(i)-pi/2)];
  
  err_x(i,:) =  cos(eta(i,:)).*(Xr(i,:) - x(i,:)) + sin(eta(i,:)).*(Yr(i,:) - y(i,:));
  err_y(i,:) = -sin(eta(i,:)).*(Xr(i,:) - x(i,:)) + cos(eta(i,:)).*(Yr(i,:) - y(i,:));  
  
  [v(i,:),w(i,:)] = u_k1(etheta(theta(i,:)),u(i,:),err_theta(i,:),err_x(i,:),err_y(i,:));
  x(i+1,:) = x(i,:) + v(i,:).*cos(eta(i,:))*dt;
  y(i+1,:) = y(i,:) + v(i,:).*sin(eta(i,:))*dt;
  eta(i+1,:) = eta(i,:) + w(i,:)*dt;
  
end

%% Gráficos
% Movimiento en el plano
figure(1)
hold on; grid on; axis equal
% plot(r,'linewidth',2)
plot(Xr(1:n-1,1),Yr(1:n-1,1),'k','linewidth',2)
plot(Xr(1:n-1,2),Yr(1:n-1,2),'r','linewidth',2)
plot(x(:,1),y(:,1),'g')
plot(x(:,2),y(:,2),'m')
plot(real(c(1)),imag(c(1)),'o');

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

% movimiento

% figure(100)
% h = animatedline;
% plot(r,'k','linewidth',2)
% for i = 1:n
%   plot(real(r(i)),imag(r(i)),'ko'); hold on
%   plot(x(i,1),y(i,1),'ro');
%   plot(x(i,2),y(i,2),'go');
%   
%   pause(0.01);
% end


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
Cx = 20;
Cxk = 0.1;
Ctheta = 2.5;
Cy = 10;
Cyk = 0.1;
K = 1;

norma = zeros(1,2);
copX = zeros(1,2);
copY = zeros(1,2);

for k = 1:2
  norma(1,k) = norm(etheta);
  
  for j = 1:2
    copX(1,k) = copX(1,k) + (err_x(k) - err_x(j));
    copY(1,k) = copY(1,k) + (err_y(k) - err_y(j));
  end
end

% v = norma.*cos(err_theta) + Cx*(err_x + Cxk*copX);
v = norma + Cx*(err_x + Cxk*copX);

k1 = zeros(1,length(etheta));
for i = 1:length(etheta)
  if err_theta(i) <= 1e-6
    k1(1,i) = 1;
  else
    k1(1,i) = sin(err_theta(i))./err_theta(i);
  end
end

alpha = sqrt(K^2 + err_x.^2 + err_y.^2);

w = theta_d + Ctheta.*err_theta + (norma.*(k1)).*(K./alpha).*Cy.*(err_y + Cyk*copY);

end
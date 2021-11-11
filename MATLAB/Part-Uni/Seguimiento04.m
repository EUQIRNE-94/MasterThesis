%% Prueba de control de seguimiento a Sepulchre
% N partículas con i = N Uniciclos en formación circular
% Ecuación (19) Sepulchre [0-02]

clear
clc
close all

%% Variables Fijas
w0 = 1;
rev = 10;
N = 4;
i = 2*N;
dt = 0.05;
tspan = 0:dt:rev*(2*pi/w0);
n = length(tspan);
kc = w0;
K = w0;

%% Sistema
%   Prealocacion de variables
% Sepulchre
r = zeros(n,N); theta = zeros(n,N); u = zeros(n-1,N); c = zeros(n-1,N);
R = zeros(n,1);
% Uniciclo
x = zeros(n,i); y = zeros(n,i); eta = zeros(n,i); Xr = zeros(n,i); Yr = zeros(n,i);
v = zeros(n-1,i); w = zeros(n-1,i);
err_theta = zeros(n-1,i); err_x = zeros(n-1,i); err_y = zeros(n-1,i);

% Condiciones iniciales sistema Sepulchre
r0 = 5*rand(1,N) + 5i*rand(1,N);
theta0 = 2*pi*rand(1,N);

% Condiciones iniciales de uniciclos
r0_2 = repmat(r0,2,1);
r0_2 = reshape(r0_2,1,[]);
theta0_2 = repmat(theta0,2,1);
theta0_2 = reshape(theta0_2,1,[]);

x0 = real(r0_2) + (.1*rand(1,i) - .05);
y0 = imag(r0_2) + (.1*rand(1,i) - .05);
eta0 = theta0_2 + (.1*rand(1,i) - .05);

% Integración del sistema
r(1,:) = r0;
theta(1,:) = theta0;
x(1,:) = x0;
y(1,:) = y0;
eta(1,:) = eta0;

for k = 1:n-1
  % Sistema de Sepulchre
  r(k+1,:) = r(k,:) + etheta(theta(k,:))*dt;
  
  c(k,:) = r(k,:) + (1i/w0)*etheta(theta(k,:));
  R(k,1) = (1/N)*sum(r(k,:));
  
  u(k,:) = u_k(w0,r(k,:),R(k,:),theta(k,:),K,kc,N);
  theta(k+1,:) = theta(k,:) + u(k,:)*dt;
  
  % Seguimiento del carrito
    % Aumentar dimension de vector referencia
  r_2 = repmat(r(k,:),2,1);
  r_2 = reshape(r_2,1,[]);
  theta_2 = repmat(theta(k,:),2,1);
  theta_2 = reshape(theta_2,1,[]);
  % Deltas de posicion
  deltatheta = zeros(1,i);
  thetar = theta_2 - deltatheta;
  
  deltaX = [zeros(1,N);.5*cos(theta(k,:) - pi/2)];
  deltaX = reshape(deltaX,1,[]);
  Xr(k,:) = real(r_2) + deltaX;
  
  deltaY = [zeros(1,N);.5*sin(theta(k,:) - pi/2)];
  deltaY = reshape(deltaY,1,[]);
  Yr(k,:) = imag(r_2) + deltaY;
  
  % Errores
  err_theta(k,:) = thetar - eta(k,:);
  err_x(k,:) =  cos(eta(k,:)).*(Xr(k,:) - x(k,:)) + sin(eta(k,:)).*(Yr(k,:) - y(k,:));
  err_y(k,:) = -sin(eta(k,:)).*(Xr(k,:) - x(k,:)) + cos(eta(k,:)).*(Yr(k,:) - y(k,:));
  
  % Perturbacion
  if k == round(n/2)
    perturbado = randi([1,i]);
    x(k,perturbado) = x(k,perturbado) + .5*cos(theta_2(perturbado) - pi/2);
    y(k,perturbado) = y(k,perturbado) + .5*sin(theta_2(perturbado) - pi/2);
  end
  
  [v(k,:),w(k,:)] = u_k1(etheta(theta(k,:)),u(k,:),err_theta(k,:),err_x(k,:),err_y(k,:));
  x(k+1,:) = x(k,:) + v(k,:).*cos(eta(k,:))*dt;
  y(k+1,:) = y(k,:) + v(k,:).*sin(eta(k,:))*dt;
  eta(k+1,:) = eta(k,:) + w(k,:)*dt;
  
  if k == n-1
    % Calculo de variables espaciales
    c(k+1,:) = r(k+1,:) + (1i/w0)*etheta(theta(k+1,:));
    R(k+1,1) = (1/N)*sum(r(k+1,:));
    
    
    r_2 = repmat(r(k+1,:),2,1);
    r_2 = reshape(r_2,1,[]);
  
    deltaX = [zeros(1,N);.5*cos(theta(k+1,:) - pi/2)];
    deltaX = reshape(deltaX,1,[]);
    Xr(k+1,:) = real(r_2) + deltaX;
  
    deltaY = [zeros(1,N);.5*sin(theta(k+1,:) - pi/2)];
    deltaY = reshape(deltaY,1,[]);
    Yr(k+1,:) = imag(r_2) + deltaY;
    
  end
end

%% Gráficos
% Movimiento en el plano
figure(1)
hold on; grid on; axis equal
plot(Xr,Yr,'linewidth',2)
plot(Xr(n,:),Yr(n,:),'go','Markersize',10,'linewidth',4)
plot(real(R(n)),imag(R(n)),'r.','Markersize',30)
plot(real(R(n)),imag(R(n)),'kx','linewidth',2,'Markersize',10)
plot(real(c(n,:)),imag(c(n,:)),'g.','Markersize',20)
plot(real(c(n,:)),imag(c(n,:)),'m*','Markersize',5)
plot(x,y)
plot(x(n,:),y(n,:),'ko','linewidth',3,'Markersize',5)
xmin = x(n-1,:);
ymin = y(n-1,:);
xmax = x(n,:);
ymax = y(n,:);
quiver(xmin,ymin,(xmax-xmin),(ymax-ymin),'k')

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
function u = u_k(w0,r,R,theta,K,k,N)
rt = r-R;
rp = etheta(theta);

dp = real(rt).*real(rp) + imag(rt).*imag(rp);

U1 = w0*(1 + k*dp);

s_theta = sin(theta);
c_theta = cos(theta);

U2 = sum(c_theta)*eye(N)*s_theta' - sum(s_theta)*eye(N)*c_theta';
U2 = ((K-k)/N)*U2';

u = U1 + U2;
end

% Morales//Nijmeier//Gutierrez
function [v,w] = u_k1(etheta,theta_d,err_theta,err_x,err_y)
Cx = 10;
Cxk = 0.1;
Ctheta = 0.1;
Cy = 0.5;
Cyk = 0.1;
K = 0.5;

etheta_2 = repmat(etheta,2,1);
etheta_2 = reshape(etheta_2,1,[]);

norma = zeros(1,length(etheta_2));
copX = zeros(1,length(etheta_2));
copY = zeros(1,length(etheta_2));

for k = 1:length(etheta_2)
  norma(1,k) = norm(etheta_2(k));
   
  for j = 1:length(etheta)
    copX(1,k) = copX(1,k) + (err_x(k) - err_x(j));
    copY(1,k) = copY(1,k) + (err_y(k) - err_y(j));
  end
end

% v = norma.*cos(err_theta) + Cx*(err_x + Cxk*copX);
v = norma + Cx*(err_x + Cxk*copX);

k1 = zeros(1,length(etheta));
for i = 1:length(etheta_2)
  if err_theta(i) <= 1e-6
    k1(1,i) = 1;
  else
    k1(1,i) = sin(err_theta(i))./err_theta(i);
  end
end

alpha = sqrt(K^2 + err_x.^2 + err_y.^2);

theta_d_2 = repmat(theta_d,2,1);
theta_d_2 = reshape(theta_d_2,1,[]);

w = theta_d_2 + Ctheta.*err_theta + (norma.*(k1)).*(K./alpha).*Cy.*(err_y + Cyk*copY);

end
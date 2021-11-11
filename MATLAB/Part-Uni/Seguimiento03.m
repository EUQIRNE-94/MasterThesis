%% Prueba de control de seguimiento a Sepulchre
% N partículas con N Uniciclos en formación circular
% Ecuación (19) Sepulchre [0-02]

clear
clc
close all

%% Variables Fijas
w0 = 1;
rev = 5;
N = 6;
dt = 0.05;
% tspan = 0:dt:rev*(2*pi/w0);
tspan = 0:dt:75;
n = length(tspan);
kc = w0;
K = w0;

%% Sistema
%   Prealocacion de variables
% Sepulchre
r = zeros(n,N); theta = zeros(n,N); u = zeros(n-1,N); c = zeros(n-1,N);
R = zeros(n,1);
% Uniciclo
x = zeros(n,N); y = zeros(n,N); eta = zeros(n,N);
v = zeros(n-1,N); w = zeros(n-1,N);
err_theta = zeros(n-1,N); err_x = zeros(n-1,N); err_y = zeros(n-1,N);

% Condiciones iniciales sistema Sepulchre
% r0 = (10*rand(1,N)-5) + 1i*(10*rand(1,N)-5);
% theta0 = 2*pi*rand(1,N);

%%% 6 Particulas
r0 = [1.8921-3.4762i,2.4815+3.2582i,-0.4946+0.3834i,-4.1618+4.9613i,-2.7102-4.2182i,4.1334-0.5732i];
theta0 = [0.6701,6.0438,0.0291,4.8689,5.1353,5.4582];

% Condiciones iniciales de uniciclos
x0 = real(r0(1,1:N)) + (.5*rand(1,N) - .25);
y0 = imag(r0(1,1:N)) + (.5*rand(1,N) - .25);
eta0 = theta0(1,1:N) + (.5*rand(1,N) - .25);

% Integración del sistema
r(1,:) = r0(1,1:N);
theta(1,:) = theta0(1,1:N);
x(1,:) = x0;
y(1,:) = y0;
eta(1,:) = eta0;

for i = 1:n-1
  % Sistema de Sepulchre
  r(i+1,:) = r(i,:) + etheta(theta(i,:))*dt;
  
  c(i,:) = r(i,:) + (1i/w0)*etheta(theta(i,:));
  R(i,1) = (1/N)*sum(r(i,:));
  
  u(i,:) = u_k(w0,r(i,:),0,theta(i,:),K,kc,N);
  theta(i+1,:) = theta(i,:) + u(i,:)*dt;
  
  % Seguimiento del carrito
  err_theta(i,:) = theta(i,:) - eta(i,:);
  err_x(i,:) =  cos(eta(i,:)).*(real(r(i,:)) - x(i,:)) + sin(eta(i,:)).*(imag(r(i,:)) - y(i,:));
  err_y(i,:) = -sin(eta(i,:)).*(real(r(i,:)) - x(i,:)) + cos(eta(i,:)).*(imag(r(i,:)) - y(i,:));
  
  [v(i,:),w(i,:)] = u_k1(etheta(theta(i,:)),u(i,:),err_theta(i,:),err_x(i,:),err_y(i,:));
  x(i+1,:) = x(i,:) + v(i,:).*cos(eta(i,:))*dt;
  y(i+1,:) = y(i,:) + v(i,:).*sin(eta(i,:))*dt;
  eta(i+1,:) = eta(i,:) + w(i,:)*dt;
  
  if i == n-1
    % Calculo de variables espaciales
    c(i+1,:) = r(i+1,:) + (1i/w0)*etheta(theta(i+1,:));
    R(i+1,1) = (1/N)*sum(r(i+1,:));
  end
end

%% Gráficos
% Movimiento en el plano
g = figure(1);
g.Resize = 'off';
set(gcf, 'Position',  [0, 0, 1000, 1000])

hold on; grid on; axis equal
plot(x,y)
for j = 1:N
  uni(x(n,j),y(n,j),eta(n,j),0.2,'blue');
  text(x(n,j)+.2,y(n,j)+.2,num2str(j))
end
plot(x(n,:),y(n,:),'ko','linewidth',3,'Markersize',5)

% plot(real(R(n)),imag(R(n)),'r.','Markersize',30)
% plot(real(R(n)),imag(R(n)),'kx','linewidth',2,'Markersize',10)
% plot(real(c(n,:)),imag(c(n,:)),'g.','Markersize',20)
% plot(real(c(n,:)),imag(c(n,:)),'m*','Markersize',5)
% xmin = real(r(n-1,:));
% ymin = imag(r(n-1,:));
% xmax = real(r(n,:));
% ymax = imag(r(n,:));
% quiver(xmin,ymin,(xmax-xmin),(ymax-ymin),'k')

% Guardar imagen
% txt0 = "img/seg_03_0" + num2str(N) + ".png";
% saveas(gcf,txt0);

% %%%% Errores
% g = figure(2);
% g.Resize = 'off';
% set(gcf, 'Position',  [0, 0, 1000, 1000])
% 
% subplot(3,1,1)
% title('Errors')
% hold on; grid on
% plot(tspan(1:n-1),err_x,'linewidth',2)
% % xlabel('Tiempo [t]')
% xlabel('Time [t]')
% ylabel('Error X')
% 
% subplot(3,1,2)
% hold on; grid on
% plot(tspan(1:n-1),err_y,'linewidth',3)
% % xlabel('Tiempo [t]')
% xlabel('Time [t]')
% ylabel('Error Y')
% 
% subplot(3,1,3)
% hold on; grid on
% plot(tspan(1:n-1),err_theta,'linewidth',3)
% % xlabel('Tiempo [t]')
% xlabel('Time [t]')
% ylabel('Error \theta')
% 
% % Guardar imagen
% txt1 = "img/err_03_0" + num2str(N) + ".png";
% saveas(gcf,txt1);
% 
% %%%% Entradas de control
% g = figure(3);
% g.Resize = 'off';
% set(gcf, 'Position',  [0, 0, 1000, 1000])
% 
% subplot(3,1,1)
% hold on; grid on
% % title('Entrada de control')
% title('Control Input')
% plot(tspan(1:n-1),u,'linewidth',3)
% % xlabel('Tiempo [t]')
% xlabel('Time [t]')
% ylabel('u')
% 
% subplot(3,1,2)
% hold on; grid on
% % title('Velocidad lineal')
% title('Lineal Velocity')
% plot(tspan(1:n-1),v,'linewidth',3)
% % xlabel('Tiempo [t]')
% xlabel('Time [t]')
% ylabel('v')
% 
% subplot(3,1,3)
% hold on; grid on
% % title('Velocidad angular')
% title('Angular Velocity')
% plot(tspan(1:n-1),w,'linewidth',3)
% % xlabel('Tiempo [t]')
% xlabel('Time [t]')
% ylabel('\omega')
% 
% % Guardar imagen
% txt1 = "img/con_03_0" + num2str(N) + ".png";
% saveas(gcf,txt1);

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
% u = U1;
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

for k = 1:length(etheta)
  norma(1,k) = norm(etheta(k));
   
  for j = 1:length(etheta)
    copX(1,k) = copX(1,k) + (err_x(k) - err_x(j));
    copY(1,k) = copY(1,k) + (err_y(k) - err_y(j));
  end
end

v = norma.*cos(err_theta) + Cx*(err_x + Cxk*copX);

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

%%% Funciones para video
function pent = uni(x,y,theta,escala,color)
xp = escala*[-1,1,1.5,1,-1];
yp = escala*[-1,-1,0,1,1];

rot = [cos(theta),-sin(theta);
  sin(theta),cos(theta)];

pos = rot*([xp;yp]) + [x;y]*ones(1,5);

pent = patch(pos(1,:),pos(2,:),color);
end
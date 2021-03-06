%% Part?culas con Din?mica de Oscilador Acoplado Discreto Sin formaciones circulares
% Ecuaci?n (8) Sepulchre [0-02]
% Ecuacion (7) Freitas [0]

% Limpieza de variables alocadas previamente
clear
close all
clc

% Variables Fijas
w0 = 0.1;
N = 6;
K = w0;
rangox = 25;
rangoy = 25;

% Tiempo
dt = 0.05;
rev = 5;
tspan = 0:dt:rev*(2*pi/w0);
n = length(tspan);

% Prealoocacion
rk = zeros(n,N);
ck = zeros(n,N);
thetak = zeros(n,N);
uk = zeros(n,N);
rk_0 = zeros(1,N);
theta_0 = zeros(1,N);

% Condiciones iniciales
for j = 1:N
  rk_0(1,j) = (2*rangox*rand() - rangox) + (2*rangoy*rand() - rangoy)*1i;
  theta_0(1,j) = 2*pi*rand();
end

% Integrador
rk(1,:) = rk_0;
thetak(1,:) = theta_0;
e_theta0 = cos(theta_0) + 1i*sin(theta_0);
ck(1,:) = rk_0 + (1i/w0)*e_theta0;

for k = 1:n-1
  uk(k,:) = u_k(w0,thetak(k,:),N,K);
  rk (k+1,:) = rk(k,:) + e_theta(thetak(k,:))*dt;
  thetak(k+1,:) = thetak(k,:) + uk(k,:)*dt;
  
  etheta = cos(thetak(k+1,:)) + 1i*sin(thetak(k+1,:));
  ck(k+1,:) = rk(k+1,:) + (1i/w0)*etheta;
end

R = (1/N)*sum(rk,2);


figure(1)
% Abrir figura a pantalla completa
% set(gcf, 'Position', get(0, 'Screensize'));
% 
plot(real(rk),imag(rk),'linewidth',1); hold on
plot(real(rk(n,:)),imag(rk(n,:)),'go','linewidth',3,'Markersize',5);
plot(real(R(n)),imag(R(n)),'r.','Markersize',30)
plot(real(R(n)),imag(R(n)),'kx','linewidth',2,'Markersize',10)

if w0 ~= 0
  plot(real(ck),imag(ck),'k')
  plot(real(ck(n,:)),imag(ck(n,:)),'g.','Markersize',20)
  plot(real(ck(n,:)),imag(ck(n,:)),'m*','Markersize',5)
end
xmin = real(rk(n-1,:));
ymin = imag(rk(n-1,:));
xmax = real(rk(n,:));
ymax = imag(rk(n,:));
quiver(xmin,ymin,(xmax-xmin),(ymax-ymin))
axis equal
grid on

% Guardar imagen
% set(gca,'LooseInset',get(gca,'TightInset'));
% saveas(gcf,'img/0_02_[8]d.png')

%% Funciones
function y = e_theta(theta)
y = cos(theta) + 1i*sin(theta);
end

function u = u_k(w0,theta,N,K)
s_theta = sin(theta);
c_theta = cos(theta);

U1 = sum(c_theta)*eye(N)*s_theta' - sum(s_theta)*eye(N)*c_theta';

u = w0 - (K/N)*U1';
end
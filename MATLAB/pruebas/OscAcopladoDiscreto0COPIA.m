%% Partículas con Dinámica de Oscilador Acoplado Discreto Sin formaciones circulares
% Ecuación (8) Sepulchre [0-02]
% Ecuacion (7) Freitas [0]

% Limpieza de variables alocadas previamente
clear
close all
clc

% Condiciones iniciales
w0 = 1;
N = 3;
K = 5;

% Tiempo
dt = 0.01;
tspan = 0:dt:10;
n = length(tspan);

% Prealoocacion
rk = zeros(n,N);
ck = zeros(n,N);
thetak = zeros(n,N);
uk = zeros(n,N);

% Condiciones iniciales
rangox = 4;
rangoy = 4;
for j = 1:N
  rk_0(1,j) = (2*rangox*rand() - rangox) + (2*rangoy*rand() - rangoy)*1i;
  theta_0(1,j) = 2*pi*rand();
end
rk(1,:) = rk_0;
thetak(1,:) = theta_0;
e_theta0 = cos(theta_0) + 1i*sin(theta_0);
ck(1,:) = rk_0 + (1i/w0)*e_theta0;

for k = 1:n-1
  rk (k+1,:) = rk(k,:) + e_theta(thetak(k,:))*dt;

  uk(k,:) = u_k(w0,thetak(k,:),N,K);
  thetak(k+1,:) = thetak(k,:) + uk(k,:)*dt;
  
  etheta = cos(thetak(k+1,:)) + 1i*sin(thetak(k+1,:));
  ck(k+1,:) = rk(k+1,:) + (1i/w0)*etheta;
end

R = (1/N)*sum(rk,2);




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
% set(gca,'LooseInset',get(gca,'TightInset'));
% saveas(gcf,'0_02_Fig2c.png')

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
%% Partículas con Dinámica de Oscilador Acoplado Discreto en formacion circular
% Ecuación (19) Sepulchre [0-02]

% Limpieza de variables alocadas previamente
clear
close all
clc

% Variables Fijas
N = 4;
w0 = 1;
K = w0;
k = w0;
rangox = 2;
rangoy = 2;

% Tiempo
dt = 0.05;
tspan = 0:dt:50;
n = length(tspan);

% Prealocacion
rk = zeros(n,N);
R = zeros(n,1);
ck = zeros(n,N);
thetak = zeros(n,N);
uk = zeros(n,N);
rk_0 = zeros(1,N);
theta_0 = zeros(1,N);

% Condiciones iniciales
% for j = 1:N
%   rk_0(1,j) = (2*rangox*rand() - rangox) + (2*rangoy*rand() - rangoy)*1i;
%   theta_0(1,j) = 2*pi*rand();
% end

rk_0 = [0.7569 + 0.9926i,-1.6647-1.0841i,-1.3905+1.3033i,1.9845-1.6873i];
theta_0 = [3.1790,6.0274,0.9380,1.5977];

% Integrador
rk(1,:) = rk_0;
thetak(1,:) = theta_0;
for T = 1:n-1
  % Estado 1
  rk (T+1,:) = rk(T,:) + e_theta(thetak(T,:))*dt;
  % Calculo de variables espaciales
  ck(T,:) = rk(T,:) + (1i/w0)*e_theta(thetak(T,:));
  R(T,1) = (1/N)*sum(rk(T,:));
  % Control
  uk(T,:) = u_k(w0,rk(T,:),R(T,:),thetak(T,:),K,k,N);
  % Estado 2
  thetak(T+1,:) = thetak(T,:) + uk(T,:)*dt;
  
  if T == n-1
    % Calculo de variables espaciales
    ck(T+1,:) = rk(T+1,:) + (1i/w0)*e_theta(thetak(T+1,:));
    R(T+1,1) = (1/N)*sum(rk(T+1,:),2);
  end
end

figure(1)
plot(real(rk),imag(rk),'linewidth',1); hold on
plot(real(rk(n,:)),imag(rk(n,:)),'ko','linewidth',3,'Markersize',5);
plot(real(R(n)),imag(R(n)),'r.','Markersize',30)
plot(real(R(n)),imag(R(n)),'kx','linewidth',2,'Markersize',10)
plot(real(R),imag(R),'k')
% plot(real(ck),imag(ck),'k')
plot(real(ck(n,:)),imag(ck(n,:)),'g.','Markersize',20)
plot(real(ck(n,:)),imag(ck(n,:)),'m*','Markersize',5)
xmin = real(rk(n-1,:));
ymin = imag(rk(n-1,:));
xmax = real(rk(n,:));
ymax = imag(rk(n,:));
quiver(xmin,ymin,(xmax-xmin),(ymax-ymin),'k')
axis equal; grid on

% Guardar imagen
% saveas(gcf,'img/0_02_[19].png')

%% Funciones
function y = e_theta(theta)
y = cos(real(theta)) + 1i*sin(real(theta));
end

function u = u_k(w0,r,R,theta,K,k,N)
rt = r-R;
% rt = r-0;
rp = e_theta(theta);

dp = real(rt).*real(rp) + imag(rt).*imag(rp);

U1 = w0*(1 + k*dp);

s_theta = sin(theta);
c_theta = cos(theta);

U2 = sum(c_theta)*eye(N)*s_theta' - sum(s_theta)*eye(N)*c_theta';
U2 = ((K-k)/N)*U2';

u = U1 + U2;
end

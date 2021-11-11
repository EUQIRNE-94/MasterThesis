%% Partículas con Dinámica de Oscilador Acoplado Discreto Sin formaciones circulares
% Ecuación (19) Sepulchre [0-02]

% Limpieza de variables alocadas previamente
clear
close all
clc

% Variables Fijas
N = 12;
w0 = .1;
K = 0.1;
k = 0.1;
rangox = 25;
rangoy = 25;

% Tiempo
dt = 0.1;
tspan = 0:dt:200-dt;
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
for j = 1:N
  rk_0(1,j) = (2*rangox*rand() - rangox) + (2*rangoy*rand() - rangoy)*1i;
  theta_0(1,j) = 2*pi*rand();
end

% Integrador
rk(1,:) = rk_0;
thetak(1,:) = theta_0;
for T = 1:n-1
  % Estado 1
  rk (T+1,:) = rk(T,:) + e_theta(thetak(T,:))*dt;
  % Calculo de variables espaciales
  ck(T,:) = rk(T,:) + (1i/w0)*e_theta(thetak(T,:));
  R(T,1) = (1/N)*sum(rk(T,:),2);
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

plot(real(rk),imag(rk),'linewidth',1); hold on
plot(real(rk(n,:)),imag(rk(n,:)),'go','linewidth',3,'Markersize',5);
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
axis equal
grid on

%% Funciones
function y = e_theta(theta)
y = cos(real(theta)) + 1i*sin(real(theta));
end

function u = u_k(w0,r,R,theta,K,k,N)
rt = r-R;
rp = e_theta(theta);

dp = real(rt).*real(rp) + imag(rt).*imag(rp);

U1 = w0*(1 + k*dp);

s_theta = sin(theta);
c_theta = cos(theta);

U2 = sum(c_theta)*eye(N)*s_theta' - sum(s_theta)*eye(N)*c_theta';
U2 = ((K-k)/N)*U2';

u = U1 + U2;
end

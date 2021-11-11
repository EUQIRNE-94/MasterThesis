%% Partículas con Dinámica de Oscilador Acoplado Discreto Sin formaciones circulares
% Ecuación (14) Freitas [0] - No funciono con el producto interno de la
% matriz de proyeccion

% Limpieza de variables alocadas previamente
clear
close all
clc

% Variables Fijas
N = 6;
M = N;
w0 = .1;
k = .1;
rangox = 15;
rangoy = 15;

% Tiempo
dt = 0.1;
tspan = 0:dt:1000-dt;
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
  uk(T,:) = u_k(w0,rk(T,:),R(T,:),thetak(T,:),k,N,M);
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
y = cos(theta) + 1i*sin(theta);
end

function u = u_k(w0,r,R,theta,k,N,M)

rt = r-R;
rp = e_theta(theta);
dp = real(rt).*real(rp) + imag(rt).*imag(rp);
U1 = w0*(1 + k*dp);

u3 = zeros(N,N,M);
% km = zeros(M,1);
% km(1:N/2,1) = w0;

km = .01*ones(M,1);
km(N) = -km(M);

for k = 1:N
  for j = 1:N
    for m = 1:M
      u3(k,j,m) = u3(k,j,m) + (km(m)/m)*sin( m*( theta(k) - theta(j) ) );
    end
  end
end

u3p = sum(u3,3);
U3 = sum(u3p,2)';

% u = U1;
u = U1 + (1/N)*U3;
end

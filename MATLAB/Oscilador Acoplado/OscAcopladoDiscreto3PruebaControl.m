%% Partículas con Dinámica de Oscilador Acoplado Discreto en Formación Simetrica Circular
% Ecuación (37) Sepulchre [0-02]
% Ecuación (16) Freitas [0]

% Limpieza de variables alocadas previamente
clear
close all
clc

% Variables Fijas
N = 12; M = N;
w0 = 0.1;
k = 0.1;
rangox = 50;  rangoy = 50;

% Tiempo
dt = 0.1;  tspan = 0:dt:350-dt;  n = length(tspan);

% Condiciones iniciales
rk_0 = zeros(1,N);  theta_0 = zeros(1,N);
for j = 1:N
  rk_0(1,j) = (2*rangox*rand() - rangox) + (2*rangoy*rand() - rangoy)*1i;
  theta_0(1,j) = 2*pi*rand();
end

% Prealocacion
rk = zeros(n,N);       R = zeros(n,1);    ck = zeros(n,N);
thetak = zeros(n,N);  uk = zeros(n,N);

% Integrador
rk(1,:) = rk_0;
thetak(1,:) = theta_0;
for T = 1:n-1
  % Estado 1
  rk (T+1,:) = rk(T,:) + e_theta(thetak(T,:))*dt;
  % Calculo de variables espaciales
  ck(T,:) = rk(T,:) + (1i/w0)*e_theta(thetak(T,:));       % Centro de particula
  R(T,1) = (1/N)*sum(rk(T,:),2);                          % Centro de masa del sistema
  % Control
  uk(T,:) = u_k(w0,rk(T,:),R(T,:),thetak(T,:),k,N,M);     % Control
  % Estado 2
  thetak(T+1,:) = thetak(T,:) + uk(T,:)*dt;
  
  if T == n-1
    % Calculo de variables espaciales
    ck(T+1,:) = rk(T+1,:) + (1i/w0)*e_theta(thetak(T+1,:));
    R(T+1,1) = (1/N)*sum(rk(T+1,:),2);
    uk(T+1,:) = u_k(w0,rk(T+1,:),R(T+1,:),thetak(T+1,:),k,N,M);
  end
end

% Prealocacion 2
rk2 = zeros(n,N);       R2 = zeros(n,1);    ck2 = zeros(n,N);
thetak2 = zeros(n,N);  uk2 = zeros(n,N);

% Integrador 2
rk2(1,:) = rk_0;
thetak2(1,:) = theta_0;
for T = 1:n-1
  % Estado 1
  rk2 (T+1,:) = rk2(T,:) + e_theta(thetak2(T,:))*dt;
  % Calculo de variables espaciales
  ck2(T,:) = rk2(T,:) + (1i/w0)*e_theta(thetak2(T,:));       % Centro de particula
  R2(T,1) = (1/N)*sum(rk2(T,:),2);                           % Centro de masa del sistema
  % Control
  uk2(T,:) = u_k2(w0,rk2(T,:),R2(T,:),thetak2(T,:),k,N,M);   % Control
  % Estado 2
  thetak2(T+1,:) = thetak2(T,:) + uk2(T,:)*dt;
  
  if T == n-1
    % Calculo de variables espaciales
    ck2(T+1,:) = rk2(T+1,:) + (1i/w0)*e_theta(thetak2(T+1,:));
    R2(T+1,1) = (1/N)*sum(rk2(T+1,:),2);
    uk2(T+1,:) = u_k2(w0,rk2(T+1,:),R2(T+1,:),thetak2(T+1,:),k,N,M);
  end
end

figure(1)
subplot(2,1,1)
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
axis equal
grid on

subplot(2,1,2)
plot(real(rk2),imag(rk2),'linewidth',1); hold on
plot(real(rk2(n,:)),imag(rk2(n,:)),'ko','linewidth',3,'Markersize',5);
plot(real(R2(n)),imag(R2(n)),'r.','Markersize',30)
plot(real(R2(n)),imag(R2(n)),'kx','linewidth',2,'Markersize',10)
plot(real(R2),imag(R2),'k')
% plot(real(ck),imag(ck),'k')
plot(real(ck2(n,:)),imag(ck2(n,:)),'g.','Markersize',20)
plot(real(ck2(n,:)),imag(ck2(n,:)),'m*','Markersize',5)
xmin = real(rk2(n-1,:));
ymin = imag(rk2(n-1,:));
xmax = real(rk2(n,:));
ymax = imag(rk2(n,:));
quiver(xmin,ymin,(xmax-xmin),(ymax-ymin),'k')
axis equal
grid on

figure(2)
subplot(2,1,1)
plot(tspan,uk,'linewidth',2)
title('Control 1 - Ecuacion 37')
grid on

subplot(2,1,2)
plot(tspan,uk2,'linewidth',2)
title('Control 2 - Ecuacion 37 Modificado')
grid on

figure(3)
plot(tspan,uk-uk2,'linewidth',2)
title('Error entre controles')
grid on

% subplot(2,1,1)
% plot(tspan,uk-w0,'linewidth',2)
% title('Error Control 1 respecto \omega_0')
% grid on
% 
% subplot(2,1,2)
% plot(tspan,uk2-w0,'linewidth',2)
% title('Error Control 2 respecto \omega_0')
% grid on

%% Funciones
function y = e_theta(theta)
y = cos(theta) + 1i*sin(theta);
end

function u = u_k(w0,r,R,theta,k,N,M)

rt = r-R;
rp = e_theta(theta);
dp = real(rt).*real(rp) + imag(rt).*imag(rp);
U1 = w0*(1 + k*dp);

s_theta = sin(theta);
c_theta = cos(theta);
U2 = sum(c_theta)*eye(N)*s_theta' - sum(s_theta)*eye(N)*c_theta';
U2 = (k/N)*U2';

u3 = zeros(N,N,M);
km = zeros(M,1);
km(1:N/2,1) = 0.1*w0;         % Reducir ganancia para mejores resultados

for p = 1:N
  for j = 1:N
    for m = 1:M
      u3(p,j,m) = u3(p,j,m) + (km(m)/m)*sin( m*( theta(p) - theta(j) ) );
    end
  end
end

u3p = sum(u3,3);
U3 = sum(u3p,2)';

u = U1 + (U2 + U3);
% u = U1 + U3;
end

%% Control 2
function u = u_k2(w0,r,R,theta,k,N,M)

rt = r-R;
rp = e_theta(theta);
dp = real(rt).*real(rp) + imag(rt).*imag(rp);
U1 = w0*(1 + k*dp);

s_theta = sin(theta);
c_theta = cos(theta);
U2 = sum(c_theta)*eye(N)*s_theta' - sum(s_theta)*eye(N)*c_theta';
U2 = (k/N)*U2';

u3 = zeros(N,N,M);
km = zeros(M,1);
km(1:N/2,1) = 0.1*w0;         % Reducir ganancia para mejores resultados

for p = 1:N
  for j = 1:N
    for m = 1:M
      u3(p,j,m) = u3(p,j,m) + (km(m)/m)*sin( m*( theta(p) - theta(j) ) );
    end
  end
end

u3p = sum(u3,3);
U3 = sum(u3p,2)';

% u = U1 + (U2 + U3);
u = U1 + U3;
end
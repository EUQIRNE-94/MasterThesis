%% Partículas con Dinámica de Oscilador Acoplado Discreto Sin formaciones circulares
% Ecuación (8) Sepulchre [0-02]
% Ecuacion (7) Freitas [0]

% Limpieza de variables alocadas previamente
clear
close all
clc

% Condiciones iniciales
w0 = 1;
N = 12;
K = 5;

% Tiempo
dt = 0.1;
tspan = 0:dt:10;
n = length(tspan);

% Prealoocacion
rk = zeros(n,N);
ck = zeros(n,N);
thetak = zeros(n,N);
uk = zeros(n,N);

% Condiciones iniciales
rangox = 10;
rangoy = 10;
for j = 1:N
  rk_0(1,j) = (2*rangox*rand() - rangox) + (2*rangoy*rand() - rangoy)*1i;
  theta_0(1,j) = 2*pi*rand();
end
rk(1,:) = rk_0;
thetak(1,:) = theta_0;

% Integración Runge-Kutta 4 orden
for i = 1:n-1
  rk(i+1,:) = runk(@(rk,thetak)fr(rk,thetak),thetak(i,:),rk(i,:),rk(i,:),dt);
  thetak(i+1,:) = runk(@(rk,thetak)ftheta(rk,thetak,w0,N,K),thetak(i,:),rk(i,:),thetak(i,:),dt);
end


R = (1/N)*sum(rk,2);

%%
plot(real(rk),imag(rk),'linewidth',1); hold on
plot(real(rk(n,:)),imag(rk(n,:)),'go','linewidth',3,'Markersize',5);
plot(real(R(n)),imag(R(n)),'r.','Markersize',30)
plot(real(R(n)),imag(R(n)),'kx','linewidth',2,'Markersize',10)
% 
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
% % set(gca,'LooseInset',get(gca,'TightInset'));
% % saveas(gcf,'0_02_Fig2c.png')

%% Funciones
% Ecuacion 1a paper [0]
function dr = fr(theta,~)

dr = cos(theta) + 1i*sin(theta);

end
% Ecuacion 1b con control (7) paper [0] o (8) paper[0-02]
function dtheta = ftheta(~,theta,w0,N,K)

for k = 1:N
  for j = 1:N
    d = sin(theta(k) - theta(j));
  end
end
dtheta = w0*ones(1,N) + (K/N)*d';
end

function y = runk(f,xk,yk,y0,dt)
% Iteracion de Runge-Kutta
k1 = f(xk,yk);
k2 = f(xk + .5*dt, yk + .5*k1*dt);
k3 = f(xk + .5*dt, yk + .5*k2*dt);
k4 = f(xk + dt, yk + k3*dt);
y = y0 + (1/6)*(k1 + 2*k2 + 2*k3 + k4)*dt;
end

function y = e_theta(theta)
y = cos(theta) + 1i*sin(theta);
end
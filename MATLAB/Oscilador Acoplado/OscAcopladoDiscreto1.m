%% Part�culas con Din�mica de Oscilador Acoplado Discreto en formacion circular
% Ecuaci�n (17) Sepulchre [0-02]

% Limpieza de variables alocadas previamente
clear
close all
clc

% Variables Fijas
N = 6;
w0 = 1;
kc = w0;
rangox = 5;
rangoy = 5;

% Tiempo
dt = 0.05;
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
for k = 1:n-1
  rk (k+1,:) = rk(k,:) + e_theta(thetak(k,:))*dt;
  
  ck(k,:) = rk(k,:) + (1i/w0)*e_theta(thetak(k,:));
  R(k,1) = (1/N)*sum(rk(k,:),2);
  
  uk(k,:) = u_k(w0,rk(k,:),R(k,:),thetak(k,:),kc,N);
  thetak(k+1,:) = thetak(k,:) + uk(k,:)*dt;
  
  if k == n-1
    ck(k+1,:) = rk(k+1,:) + (1i/w0)*e_theta(thetak(k+1,:));
    R(k+1,1) = (1/N)*sum(rk(k+1,:),2);
  end
end

figure(1)
set(gcf, 'Position', get(0, 'Screensize'));
plot(real(rk),imag(rk),'linewidth',1); hold on
plot(real(rk(n,:)),imag(rk(n,:)),'go','linewidth',3,'Markersize',5);
plot(real(R(n)),imag(R(n)),'r.','Markersize',30)
plot(real(R(n)),imag(R(n)),'kx','linewidth',2,'Markersize',10)
% plot(real(R),imag(R),'k')
% plot(real(ck),imag(ck),'k')
plot(real(ck(n,:)),imag(ck(n,:)),'g.','Markersize',20)
plot(real(ck(n,:)),imag(ck(n,:)),'m*','Markersize',5)
xmin = real(rk(n-1,:));
ymin = imag(rk(n-1,:));
xmax = real(rk(n,:));
ymax = imag(rk(n,:));
quiver(xmin,ymin,(xmax-xmin),(ymax-ymin))
axis equal
grid on
% set(gca,'LooseInset',get(gca,'TightInset'));
% saveas(gcf,'0_02_[17]b.png')

%% Funciones
function y = e_theta(theta)
y = cos(real(theta)) + 1i*sin(real(theta));
end

function u = u_k(w0,r,R,theta,kc,N)
rt = r-R;
rp = e_theta(theta);
s_theta = sin(theta);
c_theta = cos(theta);

U1 = sum(c_theta)*eye(N)*s_theta' - sum(s_theta)*eye(N)*c_theta';
U1 = (kc/N)*U1';

U2 = real(rt).*real(rp) + imag(rt).*imag(rp);

u = w0*(1 + kc*U2) + U1;
end
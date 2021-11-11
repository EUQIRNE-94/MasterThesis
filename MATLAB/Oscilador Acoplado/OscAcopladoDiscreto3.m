%% Partículas con Dinámica de Oscilador Acoplado Discreto en Formacion Circular Simetrica
% Ecuación (37) Sepulchre [0-02]

% Limpieza de variables alocadas previamente
clear
close all
clc

% Variables Fijas
N = 6;
M = N;
w0 = 1;
k = w0;
rangox = 2;
rangoy = 2;

% Tiempo
dt = 0.05;
tspan = 0:dt:75;
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

% rk_0 = [0.7569 + 0.9926i,-1.6647-1.0841i,-1.3905+1.3033i,1.9845-1.6873i];
% theta_0 = [3.1790,6.0274,0.9380,1.5977];

%%% 6 Particulas
rk_0 = [1.89-3.47i,2.48+3.25i,-0.49+0.38i,-4.16+4.96i,-2.71-4.21i,4.13-0.57i];
theta_0 = [0.67,6.04,0.02,4.86,5.13,5.45];

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

g = figure(1);
% g.Resize = 'off';
set(gcf, 'Position',  [0, 0, 800, 800])
set(gca,'FontSize',20)
axis([-2 2 -2 2])
hold on; grid on; 
% axis equal
xlabel('X','fontsize',20)
ylabel('Y','fontsize',20)

plot(real(rk),imag(rk),'k','linewidth',3);
plot(real(rk(n,:)),imag(rk(n,:)),'go','linewidth',5,'Markersize',10);
% plot(real(R(n)),imag(R(n)),'r.','Markersize',30)
% plot(real(R(n)),imag(R(n)),'kx','linewidth',2,'Markersize',10)
% plot(real(R),imag(R),'k')
% plot(real(ck),imag(ck),'k')
% plot(real(ck(n,:)),imag(ck(n,:)),'g.','Markersize',20)
plot(real(ck(n,:)),imag(ck(n,:)),'m*','Markersize',5)
xmin = real(rk(n-1,:));
ymin = imag(rk(n-1,:));
xmax = real(rk(n,:));
ymax = imag(rk(n,:));
quiver(xmin,ymin,(xmax-xmin),(ymax-ymin),'r','linewidth',3)

% Guardar imagen
% txt0 = "img/partCirc" + num2str(N) + ".png";
% saveas(gcf,txt0);

% figure(2)
% plot(tspan,uk)

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

u = U1 + U2 + U3;
end
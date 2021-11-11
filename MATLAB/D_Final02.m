%% "Movimiento Colectivo de un Sistema Multi-Agente Usando las Ecuaciones planares de Frenet-Serret"
%
% Seguimiento de particulas a formación simétrica
%
% Modelo de partículas de Sepulchre [0-02] eq.(37)
% Seguimiento de uniciclos planteado por Morales
% Evasión de colisiones en uniciclo
% Comunicación limitada en uniciclo

clc; clear; close all

%% Programa
%%%% Variables de tiempo %%%%

tf = 50;
dt = 0.05;
tspan = 0:dt:tf;
n = length(tspan);

%%%% Partículas en el plano %%%%

% r = v_0/w0;
V0 = 1;
r_0 = 2;
w0 = V0/r_0;
N = 8;
M = N;
R0 = 0;
kappa = w0;
rx = 5;
ry = 5;

%%%% Condiciones iniciales %%%%

% r0 = (2*rx*rand(1,N)-rx) + 1i*(2*ry*rand(1,N)-ry);
% theta0 = 2*pi*rand(1,N);
%%% 6 Particulas %%%
% r0 = [-2.33-0.19i,0.30+2.40i,1.90-1.71i,0.84+1.77i,-1.54+0.72i,-0.65-0.61i];
% theta0 = [0.67,6.04,0.02,4.86,5.13,5.45];
%%% 8 Particulas %%%
r0 = [4.84+4.39i,3.58-1.98i,2.85-2.04i,0.13-1.67i,-3.22-0.32i,-1.01+1.48i,-3.66-4.74i,-4.69+3.42i];
theta0 = [3.51,5.36,2.18,2.80,0.34,1.11,4.16,2.07];
%%% 10 particulas %%%
% r0 = [-4.14+4.63i,-2.37+0.46i,3.01+0.21i,-4.78-2.68i,4.28-0.11i,2.30+1.24i,-0.11+1.79i,0.78-1.04i,-2.62-1.32i,-0.41+4.87i];
% theta0 = [0.23,5.56,5.73,5.00,0.62,1.64,2.11,4.27,0.85,4.53];

%%%% Uniciclos %%%%

i = N;
d1 = 0.075;
d2 = 0.5;
K = 5;
Ks = [5,0.1,1,1,0.1,0.5];  % [Cx,Cxk,Cxi,Cy,Cyk,K]

% Condiciones iniciales
x0 = real(r0) + (.1*rand(1,i) - .05).*real(r0);
y0 = imag(r0) + (.1*rand(1,i) - .05).*imag(r0);
xi0 = theta0 + (.1*rand(1,i) - .05).*theta0;

%% Integración del sistema

%%%% Prealocacion de variables %%%%
%%% Particulas en el plano %%%
r = zeros(n,N); theta = zeros(n,N); u = zeros(n,N); c = zeros(n,N);
R = zeros(n,1);

%%% Uniciclos %%%
x = zeros(n,i); y = zeros(n,i); xi = zeros(n,i); 
Xr = zeros(n,i); Yr = zeros(n,i); XIr = zeros(n,i);
v = zeros(n,i); w = zeros(n,i);
err_xi = zeros(n,i); err_x = zeros(n,i); err_y = zeros(n,i);
rep = zeros(n,i); v_rep = zeros(n,i); d = zeros(N,N,n);

%%% Condiciones iniciales para integracion %%%
r(1,:) = r0; theta(1,:) = theta0;
x(1,:) = x0; y(1,:) = y0; xi(1,:) = xi0;

%%% Matriz de adyacencia de comunicación partículas %%%
A = ones(N);
%%% Matriz de comunicacion uniciclos %%%
Acom1 = anillo(N);
Acom2 = anilloB(N);

for k = 1:n-1
  %%% Partículas en el plano %%%
  u(k,:) = SteeringControlV2(V0,w0,r(k,:),theta(k,:),R0,kappa,N,M,A);
  [r(k+1,:),theta(k+1,:)] = ParticlesPlane(V0,r(k,:),theta(k,:),u(k,:),dt);
  
  %%% Coordenadas de referencia %%%
  Xr(k,:) = real(r(k,:));
  Yr(k,:) = imag(r(k,:));
  XIr(k,:) = theta(k,:);
  
  %%% Errores %%%
  err_xi(k,:) = XIr(k,:) - xi(k,:);
  err_x(k,:) =  cos(xi(k,:)).*(Xr(k,:) - x(k,:)) + sin(xi(k,:)).*(Yr(k,:) - y(k,:));
  err_y(k,:) = -sin(xi(k,:)).*(Xr(k,:) - x(k,:)) + cos(xi(k,:)).*(Yr(k,:) - y(k,:));
  
  %%% Uniciclos %%%
  [v(k,:),w(k,:)] = TrackingControlV2(V0,etheta(theta(k,:),V0),u(k,:),err_xi(k,:),err_x(k,:),err_y(k,:),Ks,A);
  [rep(k,:),d(:,:,k)] = evasion(x(k,:),y(k,:),d1,d2,K);
  v_rep(k,:) = rep(k,:).*v(k,:);
  [x(k+1,:),y(k+1,:),xi(k+1,:)] = Uni_Mod(x(k,:),y(k,:),xi(k,:),v_rep(k,:),w(k,:),dt);
  
end

%%%% Limpiar elementos finales %%%%
r(n,:) = []; theta(n,:) = []; R(n,:) = []; c(n,:) = []; u(n,:) = [];
x(n,:) = []; y(n,:) = []; xi(n,:) = []; rep(n,:) = []; d(:,:,n) = [];
v(n,:) = []; w(n,:) = []; v_rep(n,:) = [];
err_x(n,:) = []; err_y(n,:) = []; err_xi(n,:) = [];
tspan(n) = [];
XIr (n,:) = []; Xr (n,:) = []; Yr (n,:) = [];

%%% Límites de las figuras
maxXY = max(max(abs([x,y])));
n = n-1;

%% Animacion

% figura = figure(1);
% set(gcf, 'Position',  [100, 0, 1100, 1100])
% axis([-maxXY maxXY -maxXY maxXY])
% hold on; grid on
% for k = 1:4:n
%   
%   cla(gca(figura));
%   for j = 1:i
%     UnicycleFigure(x(k,j),y(k,j),xi(k,j),0.2,'blue');
%     text(x(k,j)+.25,y(k,j)+.25,num2str(j))
%   end
%   plot(Xr(k,:),Yr(k,:),'go','linewidth',4)
%   plot(real(r(k,:)),imag(r(k,:)),'r*','linewidth',3)
%   viscircles([x(k,:)',y(k,:)'],d1*ones(N,1));
%   viscircles([x(k,:)',y(k,:)'],(d2)*ones(N,1),'color','k');
%   text(3,3,num2str(round(tspan(k),1)))
%   text(4,3,"seg")
%   pause(0.01)
%   
% end

%% Figuras
%%% Errores %%%
figure(2);
set(gcf, 'Position',  [100, 0, 1100, 1000])
set(gca,'FontSize',20)

subplot(3,1,1)
title('Errors')
hold on; grid on
set(gca,'FontSize',20)
plot(tspan,err_x,'linewidth',2)
xlabel('Time [t]','fontsize',22)
ylabel('Error X [m]','fontsize',22)
legend({'e1','e2','e3','e4','e5','e6'},'Orientation','horizontal','fontsize',14)

subplot(3,1,2)
hold on; grid on
set(gca,'FontSize',20)
plot(tspan,err_y,'linewidth',2)
xlabel('Time [t]','fontsize',22)
ylabel('Error Y [m]','fontsize',22)
legend({'e1','e2','e3','e4','e5','e6'},'Orientation','horizontal','fontsize',14)

subplot(3,1,3)
hold on; grid on
set(gca,'FontSize',20)
plot(tspan,err_xi,'linewidth',2)
xlabel('Time [t]')
xlabel('Time [t]','fontsize',22)
ylabel('Error \theta [rad]','fontsize',22)
legend({'e1','e2','e3','e4','e5','e6','e7','e8'},'Orientation','horizontal','fontsize',14)

%%% Entradas de control %%%
figure(3);
set(gcf, 'Position',  [100, 0, 1100, 1000])
set(gca,'FontSize',20)

subplot(2,2,1)
hold on; grid on
set(gca,'FontSize',20)
title('Control input')
plot(tspan,u,'linewidth',2)
xlabel('Time [t]','fontsize',22)
ylabel('u','fontsize',22)
legend({'u_1','u_2','u_3','u_4','u_5','u_6','u_7','u_8'},'Orientation','Horizontal' ...
  ,'location','south','fontsize',14)

subplot(2,2,2)
hold on; grid on
set(gca,'FontSize',20)
title('Repulsion')
plot(tspan,rep,'linewidth',2)
xlabel('Time [t]','fontsize',22)
ylabel('Repulsion','fontsize',22)
legend({'rep_1','rep_2','rep_3','rep_4','rep_5','rep_6','rep_7','rep_8'},'Orientation','Vertical' ...
  ,'location','southeast','fontsize',14)

subplot(2,2,3)
hold on; grid on
set(gca,'FontSize',20)
title('Angular velocity')
plot(tspan,w,'linewidth',2)
xlabel('Time [t]','fontsize',22)
ylabel('\omega','fontsize',22)
legend({'\omega_1','\omega_2','\omega_3','\omega_4','\omega_5','\omega_6','\omega_7','\omega_8'}, ...
  'Orientation','Horizontal','location','south','fontsize',14)

subplot(2,2,4)
hold on; grid on
set(gca,'FontSize',20)
title('Linear velocity with repulsion')
plot(tspan,v_rep,'linewidth',2)
xlabel('Time [t]','fontsize',22)
ylabel('v','fontsize',22)
legend({'v_1','v_2','v_3','v_4','v_5','v_6','v_7','v_8'},'Orientation','Horizontal' ...
  ,'location','south','fontsize',14)

%%% Plano Fase %%%
figure(4);
set(gcf, 'Position',  [100, 0, 1100, 1000])
set(gca,'FontSize',20)

axis([-maxXY maxXY -maxXY maxXY])
hold on; grid on; axis equal
plot(Xr,Yr,'g','linewidth',3)
plot(x,y,'k--','linewidth',2)
for j = 1:N
  UnicycleFigure(x(n,j),y(n,j),xi(n,j),0.2,'blue');
end
plot(r(n,:),'ro','linewidth',5)

















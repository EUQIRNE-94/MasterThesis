%% Proyecto de Tesis "Movimiento Colectivo de un Sistema Multi-Agente Usando las Ecuaciones planares de Frenet-Serret"
%
% Programa para realizar el seguimiento a particulas en el plano con
% uniciclos utilizando comunicaciones limitadas en uniciclos
%
% Formaciones a realizar
%
%       - Patrón Simétrico
%
% Prueba de topologías en uniciclos
%       - All-to-All
%       - Anillo
%       - Anillo Bidireccional
%
% Modelo de particula y control planteado por Sepulchre en [0-02]
% Seguimiento de uniciclos planteado por Morales

clear; clc; close all

%% Programa
%%%% Variables Fijas

% Simulacion
tf = 25;
dt = 0.05;
tspan = 0:dt:tf;
n = length(tspan);
tcambio = zeros(3,1);
ncambio = zeros(3,1);
cambio = ones(3,1);

% Particulas en el plano
w0 = 1;
N = 6;
M = N;
R0 = 0;
kappa = w0;
rx = 5;
ry = 5;

%%% Condiciones iniciales
% r0 = (2*rx*rand(1,N)-rx) + 1i*(2*ry*rand(1,N)-ry);
% theta0 = 2*pi*rand(1,N);
r0 = [-2.33-0.19i,0.30+2.40i,1.90-1.71i,0.84+1.77i,-1.54+0.72i,-0.65-0.61i];
theta0 = [0.67,6.04,0.02,4.86,5.13,5.45];

%%% Uniciclos
i = N;
Ks = [10,0.1,0.5,0.5,0.1,0.5];                                                                      % [Cx,Cxk,Cxi,Cy,Cyk,K]

% Condiciones iniciales
% x0 = real(r0) + (.1*real(r0).*rand(1,i) - .05);
% y0 = imag(r0) + (.1*imag(r0).*rand(1,i) - .05);
% xi0 = theta0 + (.1*theta0.*rand(1,i) - .05);

x0 = [-2.44,0.26,2.03,0.87,-1.61,-0.76];
y0 = [-0.25,2.46,-1.89,1.74,0.70,-0.71];
xi0 = [0.67,6.56,-0.02,4.82,5.51,5.90];

%% Integracion de sistema
%%% Prealocacion de variables

%%%% Particulas en el plano
r = zeros(n,N); theta = zeros(n,N); u = zeros(n,N); c = zeros(n,N);
R = zeros(n,1);

%%%% Uniciclos All-to-All
x = zeros(n,i); y = zeros(n,i); xi = zeros(n,i); 
Xr = zeros(n,i); Yr = zeros(n,i); XIr = zeros(n,i);
v = zeros(n,i); w = zeros(n,i);
err_xi = zeros(n,i); err_x = zeros(n,i); err_y = zeros(n,i);

%%%% Uniciclos Anillo
x1 = zeros(n,i); y1 = zeros(n,i); xi1 = zeros(n,i); 
v1 = zeros(n,i); w1 = zeros(n,i);
err_xi1 = zeros(n,i); err_x1 = zeros(n,i); err_y1 = zeros(n,i);

%%%% Uniciclos Anillo Bidireccional
x2 = zeros(n,i); y2 = zeros(n,i); xi2 = zeros(n,i); 
v2 = zeros(n,i); w2 = zeros(n,i);
err_xi2 = zeros(n,i); err_x2 = zeros(n,i); err_y2 = zeros(n,i);

%%%% Variables seleccion
Cont = 6;                                                                                           % Control a utilizar en el plano (help SteeringControl)

%%%% Condiciones iniciales para integracion
r(1,:) = r0; theta(1,:) = theta0;
x(1,:) = x0; y(1,:) = y0; xi(1,:) = xi0;
x1(1,:) = x0; y1(1,:) = y0; xi1(1,:) = xi0;
x2(1,:) = x0; y2(1,:) = y0; xi2(1,:) = xi0;

A1 = anillo(N);
A2 = anilloB(N);
for k = 1:n-1
  %%% Particulas en el plano
  u(k,:) = SteeringControl(w0,r(k,:),theta(k,:),R0,kappa,N,M,Cont);                                 % Control de direccion
  [r(k+1,:),theta(k+1,:)] = ParticlesPlane(r(k,:),theta(k,:),u(k,:),dt);                            % Modelo de partículas
  
  %%% Coordenadas de referencia
  Xr(k,:) = real(r(k,:));
  Yr(k,:) = imag(r(k,:));
  XIr(k,:) = theta(k,:);
  
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Topologias comunicacion para uniciclos %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%% Errores All-to-All
  err_xi(k,:) = XIr(k,:) - xi(k,:);
  err_x(k,:) =  cos(xi(k,:)).*(Xr(k,:) - x(k,:)) + sin(xi(k,:)).*(Yr(k,:) - y(k,:));
  err_y(k,:) = -sin(xi(k,:)).*(Xr(k,:) - x(k,:)) + cos(xi(k,:)).*(Yr(k,:) - y(k,:));
  
  if norm(err_xi(k,:)) < 0.1 && norm(err_x(k,:)) < 0.1 && norm(err_y(k,:)) < 0.1 && cambio(1) == 1
    ncambio(1) = k;
    tcambio(1) = tspan(k);
    cambio(1) = 0;
  end
  
  %%% Uniciclos All-to-All
  [v(k,:),w(k,:)] = TrackingControl(etheta(theta(k,:)),u(k,:),err_xi(k,:),err_x(k,:)...
    ,err_y(k,:),Ks);                                                                                % Velocidades del uniciclo
  [x(k+1,:),y(k+1,:),xi(k+1,:)] = Uni_Mod(x(k,:),y(k,:),xi(k,:),v(k,:),w(k,:),dt);                  % Modelo de los uniciclos
  
  %%% Errores Anillo
  err_xi1(k,:) = XIr(k,:) - xi1(k,:);
  err_x1(k,:) =  cos(xi1(k,:)).*(Xr(k,:) - x1(k,:)) + sin(xi1(k,:)).*(Yr(k,:) - y1(k,:));
  err_y1(k,:) = -sin(xi1(k,:)).*(Xr(k,:) - x1(k,:)) + cos(xi1(k,:)).*(Yr(k,:) - y1(k,:));
  
  if norm(err_xi1(k,:)) < 0.1 && norm(err_x1(k,:)) < 0.1 && norm(err_y1(k,:)) < 0.1 && cambio(2) == 1
    ncambio(2) = k;
    tcambio(2) = tspan(k);
    cambio(2) = 0;
  end
  
  %%% Uniciclos Anillo
  [v1(k,:),w1(k,:)] = TrackingControlV2(etheta(theta(k,:)),u(k,:),err_xi1(k,:),err_x1(k,:)...
    ,err_y1(k,:),Ks,A1);                                                                            % Velocidades del uniciclo
  [x1(k+1,:),y1(k+1,:),xi1(k+1,:)] = Uni_Mod(x1(k,:),y1(k,:),xi1(k,:),v1(k,:),w1(k,:),dt);          % Modelo de los uniciclos
  
  %%% Errores Anillo Bidireccional
  err_xi2(k,:) = XIr(k,:) - xi2(k,:);
  err_x2(k,:) =  cos(xi2(k,:)).*(Xr(k,:) - x2(k,:)) + sin(xi2(k,:)).*(Yr(k,:) - y2(k,:));
  err_y2(k,:) = -sin(xi2(k,:)).*(Xr(k,:) - x2(k,:)) + cos(xi2(k,:)).*(Yr(k,:) - y2(k,:));
  
  if norm(err_xi2(k,:)) < 0.1 && norm(err_x2(k,:)) < 0.1 && norm(err_y2(k,:)) < 0.1 && cambio(3) == 1
    ncambio(3) = k;
    tcambio(3) = tspan(k);
    cambio(3) = 0;
  end
  
  %%% Uniciclos Anillo Bidireccional
  [v2(k,:),w2(k,:)] = TrackingControlV2(etheta(theta(k,:)),u(k,:),err_xi2(k,:),err_x2(k,:)...
    ,err_y2(k,:),Ks,A2);                                                                            % Velocidades del uniciclo
  [x2(k+1,:),y2(k+1,:),xi2(k+1,:)] = Uni_Mod(x(k,:),y(k,:),xi(k,:),v(k,:),w(k,:),dt);               % Modelo de los uniciclos
  
end
x(n,:) = []; y(n,:) = []; xi(n,:) = []; 
x1(n,:) = []; y1(n,:) = []; xi1(n,:) = [];
x2(n,:) = []; y2(n,:) = []; xi2(n,:) = [];
err_x(n,:) = []; err_y(n,:) = []; err_xi(n,:) = [];
err_x1(n,:) = []; err_y1(n,:) = []; err_xi1(n,:) = [];
err_x2(n,:) = []; err_y2(n,:) = []; err_xi2(n,:) = [];
v(n,:) = []; w(n,:) = [];
v1(n,:) = []; w1(n,:) = [];
v2(n,:) = []; w2(n,:) = [];
r(n,:) = []; theta(n,:) = []; u(n,:) = []; 
tspan(n) = [];
XIr (n,:) = []; Xr (n,:) = []; Yr (n,:) = [];
R(n,:) = []; c(n,:) = [];

%% Figuras
maxXY = max(max(abs([x,y])));
n = n-1;
%%%% 
figure(1);
set(gcf, 'Position',  [0, 0, 2000, 1000])
set(gca,'FontSize',20)

subplot(1,2,1)
title('All-to-All VS Ring')
axis([-maxXY maxXY -maxXY maxXY])
hold on; grid on; axis equal
plot(x,y,'k--','linewidth',2)
plot(x1,y1,'g--','linewidth',1)
for j = 1:N
  UnicycleFigure(x(n,j),y(n,j),xi(n,j),0.2,'blue');
  UnicycleFigure(x1(n,j),y1(n,j),xi1(n,j),0.15,'green');
end

subplot(1,2,2)
title('All-to-All VS Ring Bidirectional')
axis([-maxXY maxXY -maxXY maxXY])
hold on; grid on; axis equal
plot(x,y,'k--','linewidth',2)
plot(x2,y2,'g--','linewidth',1)
for j = 1:N
  UnicycleFigure(x(n,j),y(n,j),xi(n,j),0.2,'blue');
  UnicycleFigure(x1(n,j),y1(n,j),xi1(n,j),0.15,'green');
end

% Guardar imagen
% txt = "img/topFase" + ".png";
% saveas(gcf,txt);

%%%% Errores
figure(2);
set(gcf, 'Position',  [0, 0, 2000, 1000])
set(gca,'FontSize',20)

subplot(3,2,1)
title('Errors All-to-All VS Ring')
hold on; grid on
set(gca,'FontSize',20)
plot(tspan,err_x,'linewidth',4)
plot(tspan,err_x1,'k--','linewidth',2)
plot(tcambio(1),0,'ro','linewidth',5)
plot(tcambio(2),0,'go','linewidth',5)
xlabel('Time [t]','fontsize',22)
ylabel('Error X [m]','fontsize',22)
legend({'e1','e2','e3','e4','e5','e6'},'Orientation','horizontal','fontsize',14)

subplot(3,2,2)
title('Errors All-to-All VS Ring Bidirectional')
hold on; grid on
set(gca,'FontSize',20)
plot(tspan,err_x,'linewidth',4)
plot(tspan,err_x2,'g.','linewidth',2)
plot(tcambio(1),0,'ro','linewidth',5)
plot(tcambio(3),0,'go','linewidth',5)
xlabel('Time [t]','fontsize',22)
ylabel('Error X [m]','fontsize',22)
legend({'e1','e2','e3','e4','e5','e6'},'Orientation','horizontal','fontsize',14)

subplot(3,2,3)
hold on; grid on
set(gca,'FontSize',20)
plot(tspan,err_y,'linewidth',4)
plot(tspan,err_y1,'k--','linewidth',2)
plot(tcambio(1),0,'ro','linewidth',5)
plot(tcambio(2),0,'go','linewidth',5)
xlabel('Time [t]','fontsize',22)
ylabel('Error Y [m]','fontsize',22)
legend({'e1','e2','e3','e4','e5','e6'},'Orientation','horizontal','fontsize',14)

subplot(3,2,4)
hold on; grid on
set(gca,'FontSize',20)
plot(tspan,err_y,'linewidth',4)
plot(tspan,err_y2,'g.','linewidth',2)
plot(tcambio(1),0,'ro','linewidth',5)
plot(tcambio(3),0,'go','linewidth',5)
xlabel('Time [t]','fontsize',22)
ylabel('Error Y [m]','fontsize',22)
legend({'e1','e2','e3','e4','e5','e6'},'Orientation','horizontal','fontsize',14)

subplot(3,2,5)
hold on; grid on
set(gca,'FontSize',20)
plot(tspan,err_xi,'linewidth',4)
plot(tspan,err_xi1,'k--','linewidth',2)
plot(tcambio(1),0,'ro','linewidth',5)
plot(tcambio(2),0,'go','linewidth',5)
xlabel('Time [t]')
xlabel('Time [t]','fontsize',22)
ylabel('Error \theta [rad]','fontsize',22)
legend({'e1','e2','e3','e4','e5','e6'},'Orientation','horizontal','fontsize',14)

subplot(3,2,6)
hold on; grid on
set(gca,'FontSize',20)
plot(tspan,err_xi,'linewidth',4)
plot(tspan,err_xi2,'g.','linewidth',2)
plot(tcambio(1),0,'ro','linewidth',5)
plot(tcambio(3),0,'go','linewidth',5)
xlabel('Time [t]')
xlabel('Time [t]','fontsize',22)
ylabel('Error \theta [rad]','fontsize',22)
legend({'e1','e2','e3','e4','e5','e6'},'Orientation','horizontal','fontsize',14)

% Guardar imagen
% txt = "img/topError" + ".png";
% saveas(gcf,txt);

%%%% Entradas de control
figure(3);
set(gcf, 'Position',  [0, 0, 2000, 1000])
set(gca,'FontSize',20)

subplot(3,2,1)
hold on; grid on
set(gca,'FontSize',20)
title('Control input')
plot(tspan,u,'linewidth',2)
xlabel('Time [t]','fontsize',22)
ylabel('u','fontsize',22)
legend({'u_1','u_2','u_3','u_4','u_5','u_6'},'Orientation','Horizontal' ...
  ,'location','south','fontsize',14)

subplot(3,2,2)
hold on; grid on
set(gca,'FontSize',20)
title('Control input')
plot(tspan,u,'linewidth',2)
xlabel('Time [t]','fontsize',22)
ylabel('u','fontsize',22)
legend({'u_1','u_2','u_3','u_4','u_5','u_6'},'Orientation','Horizontal' ...
  ,'location','south','fontsize',14)

subplot(3,2,3)
hold on; grid on
set(gca,'FontSize',20)
title('Linear velocity')
plot(tspan,v,'linewidth',4)
plot(tspan,v1,'k--','linewidth',2)
plot(tcambio(1),0,'ro','linewidth',5)
plot(tcambio(2),0,'go','linewidth',5)
xlabel('Time [t]','fontsize',22)
ylabel('v','fontsize',22)
legend({'v_1','v_2','v_3','v_4','v_5','v_6'},'Orientation','Horizontal' ...
  ,'location','south','fontsize',14)

subplot(3,2,4)
hold on; grid on
set(gca,'FontSize',20)
title('Linear velocity')
plot(tspan,v,'linewidth',4)
plot(tspan,v2,'g.','linewidth',2)
plot(tcambio(1),0,'ro','linewidth',5)
plot(tcambio(3),0,'go','linewidth',5)
xlabel('Time [t]','fontsize',22)
ylabel('v','fontsize',22)
legend({'v_1','v_2','v_3','v_4','v_5','v_6'},'Orientation','Horizontal' ...
  ,'location','south','fontsize',14)

subplot(3,2,5)
hold on; grid on
set(gca,'FontSize',20)
title('Angular velocity')
plot(tspan,w,'linewidth',4)
plot(tspan,w1,'k--','linewidth',2)
plot(tcambio(1),0,'ro','linewidth',5)
plot(tcambio(2),0,'go','linewidth',5)
xlabel('Time [t]','fontsize',22)
ylabel('\omega','fontsize',22)
legend({'\omega_1','\omega_2','\omega_3','\omega_4','\omega_5','\omega_6'}, ...
  'Orientation','Horizontal','location','south','fontsize',14)

subplot(3,2,6)
hold on; grid on
set(gca,'FontSize',20)
title('Angular velocity')
plot(tspan,w,'linewidth',4)
plot(tspan,w2,'g.','linewidth',2)
plot(tcambio(1),0,'ro','linewidth',5)
plot(tcambio(3),0,'go','linewidth',5)
xlabel('Time [t]','fontsize',22)
ylabel('\omega','fontsize',22)
legend({'\omega_1','\omega_2','\omega_3','\omega_4','\omega_5','\omega_6'}, ...
  'Orientation','Horizontal','location','south','fontsize',14)

% Guardar imagen
% txt = "img/topControl" + ".png";
% saveas(gcf,txt);

%% Animacion
figura = figure(5);
set(gcf, 'Position',  [0, 0, 1000, 1000])
axis([-maxXY maxXY -maxXY maxXY])
hold on
grid on
% jj = 1;
for k = 1:2:n
  cla(gca(figura));
  for j = 1:i
    UnicycleFigure(x(k,j),y(k,j),xi(k,j),0.2,'blue');
    text(x(k,j)+.25,y(k,j)+.25,num2str(j))
  end
  plot(Xr(k,:),Yr(k,:),'go','linewidth',4)
  plot(real(r(k,:)),imag(r(k,:)),'r*','linewidth',3)
  viscircles([real(r(k,:))',imag(r(k,:))'],0.15*ones(N,1));
  text(3,3,num2str(round(tspan(k),1)))
  text(4,3,"seg")
  pause(0.05)
  
end

close(5)





















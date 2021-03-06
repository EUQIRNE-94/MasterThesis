%% Proyecto de Tesis "Movimiento Colectivo de un Sistema Multi-Agente Usando las Ecuaciones planares de Frenet-Serret"
%
% Programa para realizar subgrupos y formaciones previamente planteadas con seguimiento de uniciclos
%
% Formaciones a realizar en X subgrupos:
%       - Linea
%       - Circulo
%       - Reset
%
% Modelo de particula y control planteado por Sepulchre en [0-02]

clear; clc; close all

%% Programa
%%%% Variables Fijas

% Simulacion
tcambio = [50,100,150];
tretorno = tcambio(3);
tf = 200;
dt = 0.05;
tspan = 0:dt:tf;
n = length(tspan);

% Particulas en el plano
w0 = 0.5;
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

%%%% Uniciclos
x = zeros(n,i); y = zeros(n,i); xi = zeros(n,i); 
Xr = zeros(n,i); Yr = zeros(n,i); XIr = zeros(n,i);
v = zeros(n,i); w = zeros(n,i);
err_xi = zeros(n,i); err_x = zeros(n,i); err_y = zeros(n,i);

%%%% Variables seleccion
Cont = 6;                                                                                           % Control a utilizar en el plano (help SteeringControl)

%%%% Condiciones iniciales para integracion
r(1,:) = r0; theta(1,:) = theta0;
x(1,:) = x0; y(1,:) = y0; xi(1,:) = xi0;


for k = 1:n-1
  if tspan(k) == tretorno
    r(k,:) = x(k,:) + y(k,:)*1i;
    theta(k,:) = xi(k,:);
  end
  %%% Particulas en el plano
  u(k,:) = SteeringControl(w0,r(k,:),theta(k,:),R0,kappa,N,M,Cont);                                 % Control de direccion
  [r(k+1,:),theta(k+1,:)] = ParticlesPlane(r(k,:),theta(k,:),u(k,:),dt);                            % Modelo de part?culas
  
  %%% Coordenadas de referencia
  if tspan(k) > tcambio(1)
    if tspan(k) > tcambio(2)
      
      if tspan(k) > tretorno
        Xr(k,:) = real(r(k,:));
        Yr(k,:) = imag(r(k,:));
        XIr(k,:) = theta(k,:);
      else
        [Xr(k,:),Yr(k,:),XIr(k,:)] = SubCircleForm(r(k,:),theta(k,:),R0,2,0.5,i);
      end
      
    else
      [Xr(k,:),Yr(k,:),XIr(k,:)] = SubLinea(w0,theta(k,:),R0,2,360/(2*i),i);
    end
  else
    Xr(k,:) = real(r(k,:));
    Yr(k,:) = imag(r(k,:));
    XIr(k,:) = theta(k,:);
  end
  
  %%% Errores
  err_xi(k,:) = XIr(k,:) - xi(k,:);
  err_x(k,:) =  cos(xi(k,:)).*(Xr(k,:) - x(k,:)) + sin(xi(k,:)).*(Yr(k,:) - y(k,:));
  err_y(k,:) = -sin(xi(k,:)).*(Xr(k,:) - x(k,:)) + cos(xi(k,:)).*(Yr(k,:) - y(k,:));
  
  %%% Uniciclos
  [v(k,:),w(k,:)] = TrackingControl(etheta(theta(k,:)),u(k,:),err_xi(k,:),err_x(k,:)...
    ,err_y(k,:),Ks);                                                                                % Velocidades del uniciclo
  [x(k+1,:),y(k+1,:),xi(k+1,:)] = Uni_Mod(x(k,:),y(k,:),xi(k,:),v(k,:),w(k,:),dt);                  % Modelo de los uniciclos
  
  
  
end
x(n,:) = []; y(n,:) = []; xi(n,:) = []; r(n,:) = []; theta(n,:) = [];
u(n,:) = []; v(n,:) = []; w(n,:) = [];
err_x(n,:) = []; err_y(n,:) = []; err_xi(n,:) = [];
tspan(n) = [];
XIr (n,:) = []; Xr (n,:) = []; Yr (n,:) = [];
R(n,:) = []; c(n,:) = [];

%% Figuras

maxXY = max(max(abs([x,y])));
n = n-1;

%%%% 
figure(1);
set(gcf, 'Position',  [0, 0, 1000, 1000])
set(gca,'FontSize',20)
axis([-maxXY maxXY -maxXY maxXY])
hold on; grid on; axis equal
plot(r,'g','linewidth',3)
plot(x,y,'k--','linewidth',2)
for j = 1:N
  UnicycleFigure(x(n,j),y(n,j),xi(n,j),0.2,'blue');
end
plot(r(n,:),'ro','linewidth',5)

% Guardar imagen
% txt = "img/subCircleFase" + ".png";
% saveas(gcf,txt);

%%%% Errores
figure(2);
set(gcf, 'Position',  [0, 0, 1000, 1000])
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
legend({'e1','e2','e3','e4','e5','e6'},'Orientation','horizontal','fontsize',14)

% Guardar imagen
% txt = "img/subCircleError" + ".png";
% saveas(gcf,txt);

%%%% Entradas de control
figure(3);
set(gcf, 'Position',  [0, 0, 1000, 1000])
set(gca,'FontSize',20)

subplot(3,1,1)
hold on; grid on
set(gca,'FontSize',20)
title('Control input')
plot(tspan,u,'linewidth',2)
xlabel('Time [t]','fontsize',22)
ylabel('u','fontsize',22)
legend({'u_1','u_2','u_3','u_4','u_5','u_6'},'Orientation','Horizontal' ...
  ,'location','south','fontsize',14)

subplot(3,1,2)
hold on; grid on
set(gca,'FontSize',20)
title('Linear velocity')
plot(tspan,v,'linewidth',2)
xlabel('Time [t]','fontsize',22)
ylabel('v','fontsize',22)
legend({'v_1','v_2','v_3','v_4','v_5','v_6'},'Orientation','Horizontal' ...
  ,'location','south','fontsize',14)

subplot(3,1,3)
hold on; grid on
set(gca,'FontSize',20)
title('Angular velocity')
plot(tspan,w,'linewidth',2)
xlabel('Time [t]','fontsize',22)
ylabel('\omega','fontsize',22)
legend({'\omega_1','\omega_2','\omega_3','\omega_4','\omega_5','\omega_6'}, ...
  'Orientation','Horizontal','location','south','fontsize',14)

% Guardar imagen
% txt = "img/subCircleControl" + ".png";
% saveas(gcf,txt);

%% Animacion
% figura = figure(4);
% set(gcf, 'Position',  [0, 0, 1000, 1000])
% axis([-5.75 4.75 -5.5 5])
% hold on
% grid on
% jj = 1;
% for k = 1:3:n
%   cla(gca(figura));
%   for j = 1:i
%     UnicycleFigure(x(k,j),y(k,j),xi(k,j),0.2,'blue');
%     text(x(k,j)+.25,y(k,j)+.25,num2str(j))
%   end
%   plot(Xr(k,:),Yr(k,:),'go','linewidth',4)
%   plot(real(r(k,:)),imag(r(k,:)),'r*','linewidth',3)
%   text(3,3,num2str(round(tspan(k),1)))
%   text(4,3,"seg")
%   
% end

%% Funciones

function [Xr,Yr,XIr] = SubLinea(w0,theta,R,SubG,delta_ang,i)
% Formacion en seguimiento a varias particulas especificas
deltagamma = deg2rad((360/i) - delta_ang);

deltaX = zeros(1,i); deltaY = zeros(1,i); deltatheta = zeros(1,i);
rho = 1/w0;


%% Orden de las particulas 

if SubG >= 2 && SubG < i
  %%%% para 2 < SubG < i-1
  particulas  = zeros(1,SubG);
  Lid = ones(1,SubG);

  kk = 1;
  for k = 1:i
    particulas(kk) = particulas(kk) + 1;
    kk = kk+1;
    if kk == SubG+1
      kk = 1;
    end
  end

  for k = 2:SubG
    Lid(k) = Lid(k-1) + particulas(k-1);
  end

  deltas = theta - theta(1);
  deltas = round(deltas/(2*pi/i))-i;
  deltas = mod(deltas,-i);

  for j = 1:SubG-1
    for k = 1:i
      if abs(deltas(k)) >= (Lid(j)-1)+particulas(j) && abs(deltas(k)) < (Lid(j+1)-1)+particulas(j+1)
        deltas(k) = deltas(k) + Lid(j+1) - 1;
      end
    end
  end
else
  deltas = theta - theta(1);
  deltas = round(deltas/(2*pi/i))-i;
  deltas = mod(deltas,-i);
end


%%

for k = 1:i
  deltatheta(1,k) = theta(k) - (deltas(k))*deltagamma;
  deltaX(1,k) = rho*cos(-pi/2 + deltatheta(1,k) );
  deltaY(1,k) = rho*sin(-pi/2 + deltatheta(1,k) );
end

Xr = real(R) + deltaX;
Yr = imag(R) + deltaY;
XIr = deltatheta;

end

function [Xr,Yr,Xir] = SubCircleForm(r,theta,R,SubG,radius,i)
% Circulo alrededor de una particula especifica
a = radius;
%% Orden de las particulas 

if SubG >= 2 && SubG < i
  %%%% para 2 < SubG < i-1
  particulas  = zeros(1,SubG);
  Lid = ones(1,SubG);
  lids = ones(1,i);
  
  kk = 1;
  for k = 1:i
    particulas(kk) = particulas(kk) + 1;
    kk = kk+1;
    if kk == SubG+1
      kk = 1;
    end
  end
  
  angs = ones(1,i)*(2*pi/particulas(1));
  for k = 2:SubG
    Lid(k) = Lid(k-1) + particulas(k-1);
  end

  deltas = theta - theta(1);
  deltas = round(deltas/(2*pi/i))-i;
  deltas = mod(deltas,-i);

  for j = 1:SubG-1
    index = find(abs(deltas) == Lid(j+1)-1);
    for k = 1:i
      if abs(deltas(k)) >= (Lid(j)-1)+particulas(j) && abs(deltas(k)) < (Lid(j+1)-1)+particulas(j+1)
        deltas(k) = deltas(k) + Lid(j+1) - 1;
        lids(k) = index(1);
        angs(k) = 2*pi/particulas(j+1);
      end
    end
  end
  
else
    deltas = theta - theta(1);
    deltas = round(deltas/(2*pi/i))-i;
    deltas = mod(deltas,-i);
    lids = ones(1,i);
  angs = ones(1,i)*(2*pi/i);
end


%%

deltaX = zeros(1,i); deltaY = zeros(1,i); deltaxi = zeros(1,i);
ang = zeros(1,i);
angulos = mod(theta,2*pi); % angulo del robot

for k = 1:i
  deltaX(1,k) = real(r(lids(k))) + a*cos(theta(lids(k)) + deltas(k)*(angs(k)));
  deltaY(1,k) = imag(r(lids(k))) + a*sin(theta(lids(k)) + deltas(k)*(angs(k)));
  
  % Angulo de referencia de cada particula perpendicular a su radio con
  % respecto al centro de giro
  ang(k) = atan2(deltaY(k)-imag(R),deltaX(k)-real(R));
  if ang(k) < 0
    ang(k) = ang(k) + 2*pi;
  end
  ang(k) = ang(k) + pi/2;
  ang = mod(ang,2*pi);
  deltaxi(k) = ang(k) -  angulos(k);
  if rad2deg(deltaxi(k)) < 0
    deltaxi(k) = deltaxi(k) + 2*pi;
  end
  
Xr = deltaX;
Yr = deltaY;
Xir = theta + deltaxi;
end

end




















%% Prueba de control de seguimiento a Sepulchre
% N partículas con i = N Uniciclos en formación linea -> circular
% Ecuación (37) Sepulchre [0-02]

clear
clc
close all

%% Variables Fijas
w0 = 1;
rev = 10;
N = 6; M = N; 
i = N;
kc = w0;

% Tiempos
dt = 0.05;
% tspan = 0:dt:rev*(2*pi/w0);
tspan = 0:dt:85;
n = length(tspan);

cambio = 0;
tcambio = [0,0];
ncambio = [0,0];

%% Sistema
%   Prealocacion de variables
% Sepulchre
r = zeros(n,N); theta = zeros(n,N); u = zeros(n,N); c = zeros(n,N);
R = zeros(n,1);
% Uniciclo
x = zeros(n,i); y = zeros(n,i); eta = zeros(n,i); Xr = zeros(n,i); Yr = zeros(n,i); thetar = zeros(n,i);
v = zeros(n,i); w = zeros(n,i);
err_theta = zeros(n,i); err_x = zeros(n,i); err_y = zeros(n,i);

% Condiciones iniciales sistema Sepulchre
% r0 = (10*rand(1,N)-5) + 1i*(10*rand(1,N)-5);
% theta0 = 2*pi*rand(1,N);

%%% 4 Particulas
% r0 = [0.7569 + 0.9926i,-1.6647-1.0841i,-1.3905+1.3033i,1.9845-1.6873i];
% theta0 = [3.1790,6.0274,0.9380,1.5977];

%%% 6 Particulas
r0 = [1.8921-3.4762i,2.4815+3.2582i,-0.4946+0.3834i,-4.1618+4.9613i,-2.7102-4.2182i,4.1334-0.5732i];
theta0 = [0.6701,6.0438,0.0291,4.8689,5.1353,5.4582];

% Condiciones iniciales de uniciclos
x0 = real(r0) + (.1*rand(1,i) - .05);
y0 = imag(r0) + (.1*rand(1,i) - .05);
eta0 = theta0 + (.1*rand(1,i) - .05);

% Integracion del sistema
r(1,:) = r0; theta(1,:) = theta0; 
x(1,:) = x0; y(1,:) = y0; eta(1,:) = eta0;
k0 = 360/i; kf = k0/2; tt = (k0-kf)/2;
for k = 1:n-1
  % Sistema de Sepulchre
  r(k+1,:) = r(k,:) + etheta(theta(k,:))*dt;
  
  c(k,:) = r(k,:) + (1i/w0)*etheta(theta(k,:));
  R(k,1) = (1/N)*sum(r(k,:));

  u(k,:) = u_k(w0,r(k,:),0,theta(k,:),kc,N,M);
  theta(k+1,:) = theta(k,:) + u(k,:)*dt;
  
  % Seguimiento del carrito
  % Referencia de seguimiento
  if cambio == 0
    deltatheta = zeros(1,i);
    deltaX = zeros(1,i);
    deltaY = zeros(1,i);
  
    thetar(k,:) = theta(k,:) - deltatheta;
    Xr(k,:) = real(r(k,:)) + deltaX;
    Yr(k,:) = imag(r(k,:)) + deltaY;
  else
    if k > ncambio(1) && k <= ncambio(2)
      [Xr(k,:),Yr(k,:),thetar(k,:)] = linea(w0,theta(k,:),1,vec_theta(k-ncambio(1)),i);
    else
      [Xr(k,:),Yr(k,:),thetar(k,:)] = circulo(r(k,1),theta(k,:),1,0.5,i);
    end
  end

  % Errores
  err_theta(k,:) = thetar(k,:) - eta(k,:);
  err_x(k,:) =  cos(eta(k,:)).*(Xr(k,:) - x(k,:)) + sin(eta(k,:)).*(Yr(k,:) - y(k,:));
  err_y(k,:) = -sin(eta(k,:)).*(Xr(k,:) - x(k,:)) + cos(eta(k,:)).*(Yr(k,:) - y(k,:));
  
  % Cambio de formacion
  %%% Encontrar el momento en el que el error es minimo
  if norm(err_theta(k,:)) < 0.01 && norm(err_x(k,:)) < 0.01 && norm(err_y(k,:)) < 0.01 && cambio == 0
    ncambio = [k,k + (tt/0.05)];
    tcambio = [tspan(k),tspan(ncambio(2))];
    vec_theta = linspace(k0,kf,tt/0.05);
    cambio = 1;
  end
  
  [v(k,:),w(k,:)] = u_k1(etheta(theta(k,:)),u(k,:),err_theta(k,:),err_x(k,:),err_y(k,:));
  x(k+1,:) = x(k,:) + v(k,:).*cos(eta(k,:))*dt;
  y(k+1,:) = y(k,:) + v(k,:).*sin(eta(k,:))*dt;
  eta(k+1,:) = eta(k,:) + w(k,:)*dt;
  
  %%%% Ultima iteracion
  if k == n-1
    % Calculo de variables espaciales
    u(k+1,:) = u_k(w0,r(k+1,:),0,theta(k+1,:),kc,N,M);
    c(k+1,:) = r(k+1,:) + (1i/w0)*etheta(theta(k+1,:));
    R(k+1,1) = (1/N)*sum(r(k+1,:));

    [Xr(k+1,:),Yr(k+1,:),thetar(k+1,:)] = circulo(r(k+1,1),theta(k+1,:),1,0.5,i);
    
    % Errores
    err_theta(k+1,:) = thetar(k+1,:) - eta(k+1,:);
    err_x(k+1,:) =  cos(eta(k+1,:)).*(Xr(k+1,:) - x(k+1,:)) + sin(eta(k+1,:)).*(Yr(k+1,:) - y(k+1,:));
    err_y(k+1,:) = -sin(eta(k+1,:)).*(Xr(k+1,:) - x(k+1,:)) + cos(eta(k+1,:)).*(Yr(k+1,:) - y(k+1,:));
    
    [v(k+1,:),w(k+1,:)] = u_k1(etheta(theta(k+1,:)),u(k+1,:),err_theta(k+1,:),err_x(k+1,:),err_y(k+1,:));
  end
end

%% Figuras
% Movimiento en el plano
g = figure(1);
g.Resize = 'off';
set(gcf, 'Position',  [0, 0, 1000, 1000])
axis([-5.75 4.75 -5.5 5])
hold on; grid on; axis equal
% plot(Xr,Yr,'linewidth',2)
plot(x,y)
for j = 1:N
  uni(x(n,j),y(n,j),eta(n,j),0.2,'blue');
%   text(real(r(n,j))+.3,imag(r(n,j))+.3,num2str(j))
%   text(x(n,j)+.2,y(n,j)+.2,num2str(j))
  text(Xr(n,j)+.1,Yr(n,j)+.1,num2str(j))
end

plot(real(R(n)),imag(R(n)),'r.','Markersize',30)
plot(real(R(n)),imag(R(n)),'kx','linewidth',2,'Markersize',10)
plot(real(c(n,:)),imag(c(n,:)),'g.','Markersize',20)
plot(real(c(n,:)),imag(c(n,:)),'m*','Markersize',5)

plot(Xr(n,:),Yr(n,:),'go','Markersize',10,'linewidth',4)
plot(x(n,:),y(n,:),'ko','linewidth',3,'Markersize',5)

xmin = x(n-1,:);
ymin = y(n-1,:);
xmax = x(n,:);
ymax = y(n,:);
quiver(xmin,ymin,(xmax-xmin),(ymax-ymin),'k')

% Guardar imagen
txt0 = "img/seg_07_0" + num2str(N) + ".png";
saveas(gcf,txt0);

% Errores
figure(2)
set(gcf, 'Position',  [0, 0, 1000, 1000])
subplot(3,1,1)
title('Errores')
hold on; grid on
plot(tspan,err_x)
xlabel('Tiempo [t]')
ylabel('Error X')

subplot(3,1,2)
hold on; grid on
plot(tspan,err_y)
xlabel('Tiempo [t]')
ylabel('Error Y')

subplot(3,1,3)
hold on; grid on
plot(tspan,err_theta)
xlabel('Tiempo [t]')
ylabel('Error \theta')

% Guardar imagen
txt1 = "img/err_07_0" + num2str(N) + ".png";
saveas(gcf,txt1);

% % Entradas de control
% figure(3)
% subplot(3,1,1)
% hold on; grid on
% title('Entrada de control')
% plot(tspan,u)
% xlabel('Tiempo [t]')
% ylabel('u')
% 
% subplot(3,1,2)
% hold on; grid on
% title('Velocidad lineal')
% plot(tspan,v)
% xlabel('Tiempo [t]')
% ylabel('v')
% 
% subplot(3,1,3)
% hold on; grid on
% title('Velocidad angular')
% plot(tspan,w)
% xlabel('Tiempo [t]')
% ylabel('\omega')

%% Animacion
% Form circ
figura = figure(4);
filename = 'CircForm.gif';
set(gcf, 'Position',  [0, 0, 1000, 1000])
axis([-5.75 4.75 -5.5 5])
hold on; grid on
jj = 1;
for k = 1:2:ncambio(1)
  cla(gca(figura));
  for j = 1:i
    uni(x(k,j),y(k,j),eta(k,j),0.25,'blue');
    text(x(k,j)+.25,y(k,j)+.25,num2str(j))
  end
  plot(Xr(k,:),Yr(k,:),'go','linewidth',4)
  plot(real(r(k,:)),imag(r(k,:)),'r*','linewidth',3)
  text(3,4,num2str(round(tspan(k),1)));
  text(3.5,4,"seg");
  pause(0.01)
  
  frame(jj) = getframe(figura);
  jj = jj + 1;
end

%%% Generar Video
NAME = 'CircForm.mp4';
writerObj = VideoWriter(NAME,'MPEG-4');
open(writerObj);
writeVideo(writerObj, frame)
close(writerObj);

clear frame
close all

%Linea
figura = figure(4);
set(gcf, 'Position',  [0, 0, 1000, 1000])
axis([-5.75 4.75 -5.5 5])
hold on
grid on
jj = 1;
for k = ncambio(1)-50:1:ncambio(2)
  cla(gca(figura));
  for j = 1:i
    uni(x(k,j),y(k,j),eta(k,j),0.2,'blue');
    text(x(k,j)+.25,y(k,j)+.25,num2str(j))
  end
  plot(Xr(k,:),Yr(k,:),'go','linewidth',4)
%   if k <= ncambio(1)
    plot(real(r(k,:)),imag(r(k,:)),'r*','linewidth',3)
%   else
%     plot(real(r(k,1)),imag(r(k,1)),'r*','linewidth',3)
%   end
  if k > ncambio(2)
    plot(0,0,'g*','linewidth',3)
  end
  text(3,3,num2str(round(tspan(k),1)))
  text(4,3,"seg")
  pause(0.01)
  
  frame(jj) = getframe(figura);
  jj = jj + 1;
end

%%% Generar Video
NAME = 'LinForm.mp4';
writerObj = VideoWriter(NAME,'MPEG-4');
open(writerObj);
writeVideo(writerObj, frame)
close(writerObj);

clear frame
close all

% Ultima
figura = figure(4);
set(gcf, 'Position',  [0, 0, 1000, 1000])
axis([-5.75 4.75 -5.5 5])
hold on; grid on
jj = 1; kk = 0;
for k = ncambio(2)-50:length(tspan)
  cla(gca(figura));
  for j = 1:i
    uni(x(k,j),y(k,j),eta(k,j),0.2,'blue');
    text(x(k,j)+.25,y(k,j)+.25,num2str(j))
  end
  plot(Xr(k,:),Yr(k,:),'go','linewidth',4)
  plot(real(r(k,:)),imag(r(k,:)),'r*','linewidth',3)
%   if k <= ncambio(1)
%     plot(real(r(k,:)),imag(r(k,:)),'r*','linewidth',3)
%   else
%     plot(real(r(k,1)),imag(r(k,1)),'r*','linewidth',3)
%   end
  if tspan(k) > 35 && kk == 0
    axis([-3 3 -3 3])
    kk = 1;
  end
  if kk == 0
    text(3,3,num2str(round(tspan(k),1)),'fontsize',14)
    text(3.5,3,"seg",'fontsize',14)
  else
    text(2,2,num2str(round(tspan(k),1)),'fontsize',14)
    text(2.5,2,"seg",'fontsize',14)
  end
  pause(0.05)
  
  frame(jj) = getframe(figura);
  jj = jj + 1;
end

%%% Generar Video
NAME = 'SubCircForm.mp4';
writerObj = VideoWriter(NAME,'MPEG-4');
open(writerObj);
writeVideo(writerObj, frame)
close(writerObj);

clear frame
close all


%% Funciones
function y = etheta(theta)
y = cos(theta) + 1i*sin(theta);
end

function u = u_k(w0,r,R,theta,kc,N,M)

rt = r-R;
rp = etheta(theta);
dp = real(rt).*real(rp) + imag(rt).*imag(rp);
U1 = w0*(1 + kc*dp);

s_theta = sin(theta);
c_theta = cos(theta);
U2 = sum(c_theta)*eye(N)*s_theta' - sum(s_theta)*eye(N)*c_theta';
U2 = (kc/N)*U2';

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
end

% Morales//Nijmeier//Gutierrez
function [v,w] = u_k1(etheta,theta_d,err_theta,err_x,err_y)
Cx = 10;
Cxk = 0.1;
Ctheta = 0.1;
Cy = 0.5;
Cyk = 0.1;
K = 0.5;

norma = zeros(1,length(etheta));
copX = zeros(1,length(etheta));
copY = zeros(1,length(etheta));

for k = 1:length(etheta)
  norma(1,k) = norm(etheta(k));
   
  for j = 1:length(etheta)
    copX(1,k) = copX(1,k) + (err_x(k) - err_x(j));
    copY(1,k) = copY(1,k) + (err_y(k) - err_y(j));
  end
end

% v = norma.*cos(err_theta) + Cx*(err_x + Cxk*copX);
v = norma + Cx*(err_x + Cxk*copX);
for i = 1:length(etheta)
  if v(i) > 1.5
    v(i) = 1.5;
  end
end

k1 = zeros(1,length(etheta));
for i = 1:length(etheta)
  if err_theta(i) <= 1e-6
    k1(1,i) = 1;
  else
    k1(1,i) = sin(err_theta(i))./err_theta(i);
  end
end

alpha = sqrt(K^2 + err_x.^2 + err_y.^2);

w = theta_d + Ctheta.*err_theta + (norma.*(k1)).*(K./alpha).*Cy.*(err_y + Cyk*copY);

end

%%% Formaciones de referencia
function [Xr,Yr,thetar] = linea(w0,theta,part,delta_ang,i)
% Formacion en seguimiento a una particula especifica
deltagamma = deg2rad((360/i) - delta_ang);

deltaX = zeros(1,i); deltaY = zeros(1,i); deltatheta = zeros(1,i);
rho = 1/w0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obtener acomodo de las particulas
deltas = theta - theta(part);
deltas = mod(deltas,2*pi);
for o = 1:i
    if deltas(o) > 0
        deltas(o) = (deltas(o) - 2*pi);
    end
end
deltas = round(deltas/(2*pi/i));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for o = 1:i
  deltatheta(1,o) = theta(o) - (deltas(o))*deltagamma;
  deltaX(1,o) = rho*cos(-pi/2 + deltatheta(1,o) );
  deltaY(1,o) = rho*sin(-pi/2 + deltatheta(1,o) );
end

Xr = deltaX;
Yr = deltaY;
thetar = deltatheta;
end

function [Xr,Yr,thetar] = circulo(r,theta,part,radio,i)
% Circulo alrededor de una particula especifica
a = radio;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obtener acomodo de las particulas
deltas = theta - theta(part);
deltas = mod(deltas,2*pi);
for o = 1:i
    if deltas(o) > 0
        deltas(o) = (deltas(o) - 2*pi);
    end
end
deltas = round(deltas/(2*pi/i));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
deltaX = zeros(1,i); deltaY = zeros(1,i); deltatheta = zeros(1,i);
ang = zeros(1,i);
angulos = mod(theta,2*pi); % angulo del robot
deltangulos = angulos(part) - angulos; % Valor a sumar o restar
for o = 1:i
  deltaX(1,o) = real(r) + a*cos(theta(part) + deltas(o)*(2*pi/i));
  deltaY(1,o) = imag(r) + a*sin(theta(part) + deltas(o)*(2*pi/i));
  
  % Angulo de referencia de cada particula perpendicular a su radio con
  % respecto al centro de giro
  ang(o) = atan2(deltaY(o),deltaX(o));
  if ang(o) < 0
    ang(o) = ang(o) + 2*pi;
  end
  ang(o) = ang(o) + pi/2;
  ang = mod(ang,2*pi);
  deltatheta(o) = ang(o) -  angulos(o);
  if rad2deg(deltatheta(o)) < 0
    deltatheta(o) = deltatheta(o) + 2*pi;
  end
  if deltangulos(o) < 0
    deltangulos(o) = deltangulos(o) + 2*pi;
  end
end

Xr = deltaX;
Yr = deltaY;
% thetar = theta + deltangulos;
thetar = theta + deltatheta;
end

%%% Funciones para video
function pent = uni(x,y,theta,escala,color)
xp = escala*[-1,1,1.5,1,-1];
yp = escala*[-1,-1,0,1,1];

rot = [cos(theta),-sin(theta);
  sin(theta),cos(theta)];

pos = rot*([xp;yp]) + [x;y]*ones(1,5);

pent = patch(pos(1,:),pos(2,:),color);
end
%% Prueba de control de seguimiento a Sepulchre
% N partículas con i = N Uniciclos en formación circular
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
r0 = [1.89-3.47i,2.48+3.25i,-0.49+0.38i,-4.16+4.96i,-2.71-4.21i,4.13-0.57i];
theta0 = [0.67,6.04,0.02,4.86,5.13,5.45];

% Timepos
ncambio = n;
tcambio = tspan(ncambio);

% Condiciones iniciales de uniciclos
% Condiciones iniciales de uniciclos
x0 = real(r0) + (.1*real(r0).*rand(1,i) - .05);
y0 = imag(r0) + (.1*imag(r0).*rand(1,i) - .05);
eta0 = theta0 + (.1*theta0.*rand(1,i) - .05);

% Integracion del sistema
r(1,:) = r0; theta(1,:) = theta0; 
x(1,:) = x0; y(1,:) = y0; eta(1,:) = eta0;

for k = 1:n-1
  % Sistema de Sepulchre
  r(k+1,:) = r(k,:) + etheta(theta(k,:))*dt;
  
  c(k,:) = r(k,:) + (1i/w0)*etheta(theta(k,:));
  R(k,1) = (1/N)*sum(r(k,:));

  u(k,:) = u_k(w0,r(k,:),0,theta(k,:),kc,N,M);
  theta(k+1,:) = theta(k,:) + u(k,:)*dt;
  
  % Seguimiento del carrito
  % Referencia de seguimiento
  deltatheta = zeros(1,i);
  deltaX = zeros(1,i);
  deltaY = zeros(1,i);
  
  thetar(k,:) = theta(k,:) - deltatheta;
  Xr(k,:) = real(r(k,:)) + deltaX;
  Yr(k,:) = imag(r(k,:)) + deltaY;

  % Errores
  err_theta(k,:) = thetar(k,:) - eta(k,:);
  err_x(k,:) =  cos(eta(k,:)).*(Xr(k,:) - x(k,:)) + sin(eta(k,:)).*(Yr(k,:) - y(k,:));
  err_y(k,:) = -sin(eta(k,:)).*(Xr(k,:) - x(k,:)) + cos(eta(k,:)).*(Yr(k,:) - y(k,:));
  
  % Cambio de formacion
  %%% Encontrar el momento en el que el error es minimo
%   if norm(err_theta(k,:)) < 0.01 && norm(err_x(k,:)) < 0.01 && norm(err_y(k,:)) < 0.01 && cambio == 0
%     ncambio = k;
%     tcambio = tspan(k);
%     cambio = 1;
%   end
  
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
    
    deltatheta = zeros(1,i);
    deltaX = zeros(1,i);
    deltaY = zeros(1,i);
  
    thetar(k+1,:) = theta(k+1,:) + deltatheta;
    Xr(k+1,:) = real(r(k+1,:)) + deltaX;
    Yr(k+1,:) = imag(r(k+1,:)) + deltaY;
    
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
% g.Resize = 'off';
set(gcf, 'Position',  [0, 0, 1000, 1000])
set(gca,'FontSize',20)
axis([-2 2 -2 2])
hold on; grid on; 
% axis equal
xlabel('X','fontsize',20)
ylabel('Y','fontsize',20)

plot(Xr(1:ncambio,:),Yr(1:ncambio,:),'k','linewidth',4)
plot(x(1:ncambio,:),y(1:ncambio,:),'g--','linewidth',2)
for j = 1:N
  uni(x(ncambio,j),y(ncambio,j),eta(ncambio,j),0.2,'blue');
%   text(x(ncambio,j)+.25,y(ncambio,j)+.25,num2str(j),'fontsize',22)
end
plot(Xr(ncambio,:),Yr(ncambio,:),'ro','Markersize',10,'linewidth',4)

% % Guardar imagen
% txt0 = "img/circFormBase" + num2str(N) + ".png";
% saveas(gcf,txt0);

%%%% Errores
g = figure(2);
g.Resize = 'off';
set(gcf, 'Position',  [0, 0, 1000, 1000])
subplot(3,1,1)
title('Errors')
hold on; grid on
set(gca,'FontSize',20)
axis([0 85 -1 1])
plot(tspan,err_x,'linewidth',2)
% xlabel('Tiempo [t]')
xlabel('Time [t]','fontsize',22)
ylabel('Error X [m]','fontsize',22)
legend({'e1','e2','e3','e4','e5','e6'},'Orientation','horizontal','fontsize',14)

subplot(3,1,2)
hold on; grid on
set(gca,'FontSize',20)
axis([0 85 -1 1])
plot(tspan,err_y,'linewidth',2)
% xlabel('Tiempo [t]')
xlabel('Time [t]','fontsize',22)
ylabel('Error Y [m]','fontsize',22)
legend({'e1','e2','e3','e4','e5','e6'},'Orientation','horizontal','fontsize',14)

subplot(3,1,3)
hold on; grid on
set(gca,'FontSize',20)
axis([0 85 -1 1])
plot(tspan,err_theta,'linewidth',2)
% xlabel('Tiempo [t]')
xlabel('Time [t]')
xlabel('Time [t]','fontsize',22)
ylabel('Error \theta [rad]','fontsize',22)
legend({'e1','e2','e3','e4','e5','e6'},'Orientation','horizontal','fontsize',14)

% Guardar imagen
txt1 = "circForm_errBase" + num2str(N) + ".png";
saveas(gcf,txt1);

%%%% Entradas de control
% g = figure(3);
% g.Resize = 'off';
% set(gcf, 'Position',  [0, 0, 800, 800])
% 
% subplot(3,1,1)
% hold on; grid on
% % title('Entrada de control')
% title('Control Input')
% plot(tspan,u,'linewidth',1)
% % xlabel('Tiempo [t]')
% xlabel('Time [t]')
% ylabel('u')
% legend('u_1','u_2','u_3','u_4','u_5','u_6')
% 
% subplot(3,1,2)
% hold on; grid on
% % title('Velocidad lineal')
% title('Linear Velocity')
% plot(tspan,v,'linewidth',1)
% % xlabel('Tiempo [t]')
% xlabel('Time [t]')
% ylabel('v')
% legend('v_1','v_2','v_3','v_4','v_5','v_6')
% 
% subplot(3,1,3)
% hold on; grid on
% % title('Velocidad angular')
% title('Angular Velocity')
% plot(tspan,w,'linewidth',1)
% % xlabel('Tiempo [t]')
% xlabel('Time [t]')
% ylabel('\omega')
% legend('\omega_1','\omega_2','\omega_3','\omega_4','\omega_5','\omega_6')
% 
% % Guardar imagen
% txt2 = "img/conCirc" + num2str(N) + ".png";
% saveas(gcf,txt2);

%% Animacion
% figura = figure(4);
% filename = 'CircForm.gif';
% set(gcf, 'Position',  [0, 0, 1000, 1000])
% axis([-5.75 4.75 -5.5 5])
% hold on; grid on
% jj = 1;
% for k = 1:2:ncambio+50
%   cla(gca(figura));
%   for j = 1:i
%     uni(x(k,j),y(k,j),eta(k,j),0.25,'blue');
%     text(x(k,j)+.25,y(k,j)+.25,num2str(j))
%   end
%   plot(Xr(k,:),Yr(k,:),'go','linewidth',4)
%   plot(real(r(k,:)),imag(r(k,:)),'r*','linewidth',3)
%   text(3,4,num2str(round(tspan(k),1)));
%   text(3.5,4,"seg");
%   pause(0.01)
%   
% %   frame(jj) = getframe(figura);
% %   jj = jj + 1;
% end
% 
% %%% Generar Video
% % NAME = 'img/CircForm.mp4';
% % writerObj = VideoWriter(NAME,'MPEG-4');
% % open(writerObj);
% % writeVideo(writerObj, frame)
% % close(writerObj);
% % 
% % clear frame
% close all


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

%%%% Funciones para video
function pent = uni(x,y,theta,escala,color)
xp = escala*[-1,1,1.5,1,-1];
yp = escala*[-1,-1,0,1,1];

rot = [cos(theta),-sin(theta);
  sin(theta),cos(theta)];

pos = rot*([xp;yp]) + [x;y]*ones(1,5);

pent = patch(pos(1,:),pos(2,:),color);
end
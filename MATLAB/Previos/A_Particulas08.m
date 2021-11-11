%% Proyecto de Tesis "Movimiento Colectivo de un Sistema Multi-Agente Usando las Ecuaciones planares de Frenet-Serret"
%
% Programa para realizar el movimiento de particulas en el plano utilizando
% acoplamiento senoidal y comparar el sistema en comunicación All-to-All
% con Anillo y Anillo Bidireccional
%
% Modelo de particula y control planteado por Sepulchre en [0-02]
% Control (17) All-to-All
% Control (20) Limitado

clear; clc; close all

%% Programa
%%%% Variables Fijas

% Simulacion
tf = 50;
dt = 0.05;
tspan = 0:dt:tf;
n = length(tspan);

% Particulas en el plano
w0 = 0.5;
N = 6;
M = N;
R = 0;
kappa = w0;
rx = 5;
ry = 5;

%%% Condiciones iniciales
% r0 = (2*rx*rand(1,N)-rx) + 1i*(2*ry*rand(1,N)-ry);
% theta0 = 2*pi*rand(1,N);
r_ini = [-2.33-0.19i,0.30+2.40i,1.90-1.71i,0.84+1.77i,-1.54+0.72i,-0.65-0.61i];
theta_ini = [0.67,6.04,0.02,4.86,5.13,5.45];

A1 = ones(N);
A2 = anillo(N);
A3 = anilloB(N);

%% Integracion de sistema
%%% Prealocacion de variables

%%%% Particulas en el plano All-to-All
r0 = zeros(n,N); theta0 = zeros(n,N); u0 = zeros(n,N); c0 = zeros(n,N);
R0 = zeros(n,1);


%%%% Particulas en el plano Anillo
r1 = zeros(n,N); theta1 = zeros(n,N); u1 = zeros(n,N); c1 = zeros(n,N);
R1 = zeros(n,1);

%%%% Variables seleccion
Cont = 4;                                                                                           % Control a utilizar en el plano

r0(1,:) = r_ini;
theta0(1,:) = theta_ini;
r1(1,:) = r_ini;
theta1(1,:) = theta_ini;
for k = 1:n-1
  %% All-to-All
  c0(k,:) = r0(k,:) + (1i/w0)*etheta(theta0(k,:));
  
  %%% Particulas en el plano
  u0(k,:) = SteeringControl(w0,r0(k,:),theta0(k,:),R,kappa,N,M,Cont);                               % Control de direccion
  [r0(k+1,:),theta0(k+1,:)] = ParticlesPlane(r0(k,:),theta0(k,:),u0(k,:),dt);                       % Modelo de partículas
  
  %% Anillo
  c1(k,:) = r1(k,:) + (1i/w0)*etheta(theta1(k,:));
  u1(k,:) = con_lim(w0,r1(k,:),theta1(k,:),c1(k,:),kappa,A2);
  [r1(k+1,:),theta1(k+1,:)] = ParticlesPlane(r1(k,:),theta1(k,:),u1(k,:),dt);                       % Modelo de partículas
end

%% Figuras
maxXY = max(max(abs([real(r0),imag(r0)])));
n = n-1;

figure(1);
set(gcf, 'Position',  [0, 0, 1000, 1000])
set(gca,'FontSize',20)

subplot(1,2,1)
axis([-maxXY maxXY -maxXY maxXY])
hold on; grid on; 
xlabel('X','fontsize',20)
ylabel('Y','fontsize',20)
title('Plano Fase')

plot(real(r0),imag(r0),'k','linewidth',3);
plot(real(r0(n,:)),imag(r0(n,:)),'go','linewidth',5,'Markersize',10);
plot(real(c0(n,:)),imag(c0(n,:)),'m*','Markersize',5)
xmin = real(r0(n-1,:));
ymin = imag(r0(n-1,:));
xmax = real(r0(n,:));
ymax = imag(r0(n,:));
quiver(xmin,ymin,(xmax-xmin),(ymax-ymin),'r','linewidth',3)

maxXY = max(max(abs([real(r1),imag(r1)])));
subplot(1,2,2)
axis([-maxXY maxXY -maxXY maxXY])
hold on; grid on; 
xlabel('X','fontsize',20)
ylabel('Y','fontsize',20)
title('Plano Fase Limitado')

plot(real(r1),imag(r1),'k','linewidth',3);
plot(real(r1(n,:)),imag(r1(n,:)),'go','linewidth',5,'Markersize',10);
plot(real(c1(n,:)),imag(c1(n,:)),'m*','Markersize',5)
xmin = real(r1(n-1,:));
ymin = imag(r1(n-1,:));
xmax = real(r1(n,:));
ymax = imag(r1(n,:));
quiver(xmin,ymin,(xmax-xmin),(ymax-ymin),'r','linewidth',3)

% Guardar imagen
% txt = "img/partControl37" + ".png";
% saveas(gcf,txt);

figure(2);
set(gcf, 'Position',  [0, 0, 1000, 1000])
set(gca,'FontSize',20)

subplot(1,2,1)
hold on; grid on; 
xlabel('X','fontsize',20)
ylabel('Y','fontsize',20)
title('Entrada de control')

plot(tspan(1:n),u0(1:n,:),'linewidth',3)
legend({'u_1','u_2','u_3','u_4','u_5','u_6'},'Orientation','Horizontal' ...
  ,'location','south','fontsize',14)

subplot(1,2,2)
hold on; grid on; 
xlabel('X','fontsize',20)
ylabel('Y','fontsize',20)
title('Entrada de control Limitado')

plot(tspan(1:n),u1(1:n,:),'linewidth',3)
legend({'u_1','u_2','u_3','u_4','u_5','u_6'},'Orientation','Horizontal' ...
  ,'location','south','fontsize',14)

% Guardar imagen
% txt = "img/partControl37_2" + ".png";
% saveas(gcf,txt);

%% Funciones

function res = con_lim(w0,r,theta,c,kappa,A)
  d = sum(A,2);
  
  s = -1i*w0*c;
  rr = 1i*etheta(theta);
  
  ptheta = (1./d).*(A*etheta(theta)');
%   R = (1./d).*(A*r');
  R = 0;
  p1 = s - (ptheta' - 1i*w0*R');
  
  dp = real(p1).*real(rr) + imag(p1).*imag(rr);
  
  s_theta = sin(theta);
  c_theta = cos(theta);
  u1 = diag(s_theta)*A*c_theta' - diag(c_theta)*A*s_theta';
  u1 = (kappa./(1*d)).*u1;
  
  res = w0 - kappa*dp + u1';
%   res = w0 - 0.5*kappa*dp;
%   pause(0.1)
end
















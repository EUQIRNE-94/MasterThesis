%% Proyecto de Tesis "Movimiento Colectivo de un Sistema Multi-Agente Usando las Ecuaciones planares de Frenet-Serret"
%
% Programa para realizar el movimiento de particulas en el plano utilizando
% acoplamiento senoidal
%
% Se agrega la evasion de colisiones entre particulas
%
% Modelo de particula y control planteado por Sepulchre en [0-02]
% Evasión de coisiones planteado por Freitas

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
R0 = 0;
kappa = w0;
rx = 5;
ry = 5;

%%% Condiciones iniciales
% r0 = (2*rx*rand(1,N)-rx) + 1i*(2*ry*rand(1,N)-ry);
% theta0 = 2*pi*rand(1,N);
r0 = [-2.33-0.19i,0.30+2.40i,1.90-1.71i,0.84+1.77i,-1.54+0.72i,-0.65-0.61i];
theta0 = [0.67,6.04,0.02,4.86,5.13,5.45];

%% Integracion de sistema
%%% Prealocacion de variables

%%%% Particulas en el plano
r = zeros(n,N); theta = zeros(n,N); u = zeros(n,N); c = zeros(n,N);
R = zeros(n,1); rep = zeros(n,N);

%%%% Variables seleccion
Cont = 6;                                                                                           % Control a utilizar en el plano

r(1,:) = r0;
theta(1,:) = theta0;
for k = 1:n-1
  c(k,:) = r(k,:) + (1i/w0)*etheta(theta(k,:));
  %%% Particulas en el plano
  u(k,:) = SteeringControl(w0,r(k,:),theta(k,:),R0,kappa,N,M,Cont);                                 % Control de direccion
  rep(k,:) = evasion(r(k,:),theta(k,:),0.5,0.25,5);
  
  [r(k+1,:),theta(k+1,:)] = ParticlesPlane(r(k,:),theta(k,:),u(k,:)+rep(k,:),dt);                            % Modelo de partículas
  
end

%% Figuras
maxXY = max(max(abs([real(r),imag(r)])));
n = n-1;


figure(1);
set(gcf, 'Position',  [0, 0, 1000, 1000])
set(gca,'FontSize',20)
axis([-maxXY maxXY -maxXY maxXY])
hold on; grid on; 
xlabel('X','fontsize',20)
ylabel('Y','fontsize',20)
title('Plano Fase')

plot(real(r),imag(r),'k','linewidth',3);
plot(real(r(n,:)),imag(r(n,:)),'go','linewidth',5,'Markersize',10);
plot(real(c(n,:)),imag(c(n,:)),'m*','Markersize',5)
xmin = real(r(n-1,:));
ymin = imag(r(n-1,:));
xmax = real(r(n,:));
ymax = imag(r(n,:));
quiver(xmin,ymin,(xmax-xmin),(ymax-ymin),'r','linewidth',3)

% Guardar imagen
% txt = "img/partControl37" + ".png";
% saveas(gcf,txt);

figure(2);
set(gcf, 'Position',  [0, 0, 1000, 1000])
set(gca,'FontSize',20)
subplot(3,1,1)
hold on; grid on; 
xlabel('X','fontsize',20)
ylabel('Y','fontsize',20)
title('Entrada de control')

plot(tspan(1:n),u(1:n,:),'linewidth',3)
legend({'u_1','u_2','u_3','u_4','u_5','u_6'},'Orientation','Horizontal' ...
  ,'location','south','fontsize',14)

subplot(3,1,2)
hold on; grid on; 
xlabel('X','fontsize',20)
ylabel('Y','fontsize',20)
title('Repulsion')

plot(tspan(1:n),rep(1:n,:),'linewidth',3)
legend({'rep_1','rep_2','rep_3','rep_4','rep_5','rep_6'},'Orientation','Horizontal' ...
  ,'location','south','fontsize',14)

subplot(3,1,3)
hold on; grid on; 
xlabel('X','fontsize',20)
ylabel('Y','fontsize',20)
title('Control + Repulsion')

plot(tspan(1:n),u(1:n,:)+rep(1:n,:),'linewidth',3)
legend({'u_1+rep_1','u_2+rep_2','u_3+rep_3','u_4+rep_4','u_5+rep_5','u_6+rep_6'},'Orientation','Horizontal' ...
  ,'location','south','fontsize',14)

% Guardar imagen
% txt = "img/partControl37_2" + ".png";
% saveas(gcf,txt);

%% Animacion
figura = figure(4);
set(gcf, 'Position',  [0, 0, 1000, 1000])
axis([-maxXY maxXY -maxXY maxXY])
hold on; grid on

for k = 1:n
  cla(gca(figura));
  plot(real(r(k,:)),imag(r(k,:)),'r*','linewidth',3)
  viscircles([real(r(k,:))',imag(r(k,:))'],0.5*ones(N,1));
  viscircles([real(r(k,:))',imag(r(k,:))'],0.25*ones(N,1),'Color','k');
  text(3,3,num2str(round(tspan(k),1)))
  text(4,3,"seg")
  pause(0.05)
end

%% Funciones

function rep = evasion(r,theta,d1,d2,K)
  
  N = length(r);
  rep = zeros(1,N);
  count = 0;
  M = zeros(N);
  f = zeros(N);
  
  for k = 1:N
    for j = 1:N
      M(k,j) = norm(r(k) - r(j));
      f(k,j) = f_tan(d1,d2,M(k,j));
      if M(k,j) < d1
        count = count + 1;
        rep(k) = f(k,j)*(rep(k) + sin(theta(k) - theta(j)));
      end
      
    end
    rep(k) = (K/count)*rep(k);
    count = 0;
  end
%   pause(0.1);
end

function f = f_tan(d1,d2,d)
  deltaD = d1-d2;
  deltaR = d-d2;
  f = tanh(5*((deltaD-deltaR)/deltaD));
end
















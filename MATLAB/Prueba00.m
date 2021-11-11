%% Prueba de sistema de Sepulchre con matriz de adyacencia incluida

clear
close all
clc

%% Variables Fijas
%%% Sepulchre
w0 = 1;
N = 6;
M = N;
R0 = 0;
kappa = w0;
CSep = 1;
A0 = ones(N);
A1 = anillo(N);
A2 = anilloB(N);

%%% Simulacion
tf = 100;
dt = 0.05;
tspan = 0:dt:tf;
n = length(tspan);
rX = 5;
rY = 5;

%% Sistema
% Condiciones iniciales sistema Sepulchre ALEATORIAS
% r0 = (rX*rand(1,N)-(rX/2)) + 1i*(rY*rand(1,N)-(rY/2));
% theta0 = 2*pi*rand(1,N);

% Condiciones iniciales sistema Sepulchre FIJAS
%%%%%% 4 particulas
% r0 = [-2.19+0.78i,-0.50+0.63i,0.13-1.04i,-0.41-0.34i];
% theta0 = [0.0973,6.1831,1.0504,0.6674];
%%%%%% 6 particulas
r0 = [-2.33-0.19i,0.30+2.40i,1.90-1.71i,0.84+1.77i,-1.54+0.72i,-0.65-0.61i];
theta0 = [0.67,6.04,0.02,4.86,5.13,5.45];

% Condiciones iniciales de uniciclos ALEATORIAS
% x0 = real(r0) + (0.1*real(r0).*rand(1,N) - real(r0)*0.05);
% y0 = imag(r0) + (0.1*imag(r0).*rand(1,N) - imag(r0)*0.05);
% xi0 = theta0 + (0.1*theta0.*rand(1,N) - theta0*0.05);

% Condiciones iniciales de uniciclos FIJAS
%%%%%% 4 particulas
x0 = [-2.1720   -0.5242    0.1274   -0.4182];
y0 = [ 0.7930    0.6325   -1.0606   -0.3457];
xi0 = [0.0942    5.9531    1.1028    0.6455];

%   Prealocacion de variables
% Sepulchre
r = zeros(n,N); theta = zeros(n,N); u = zeros(n,N); c = zeros(n,N);
R = zeros(n,1);

%% Integracion del sistema
%%% Variables iniciales Sepulchre
r(1,:) = r0; theta(1,:) = theta0;

A = A1;
for k = 1:n-1
  % Obtener control de direccion
  [u(k,:),U1,U2,U3] = SteeringControlV2(w0,r(k,:),theta(k,:),R0,kappa,N,M,A);
  [r(k+1,:),theta(k+1,:)] = ParticlesPlane(r(k,:),theta(k,:),u(k,:),dt);
  
  %%%%% Utlima iteracion
  if k == n-1
    % Control de ultima iteracion
    [u(k+1,:),U1,U2,U3] = SteeringControlV2(w0,r(k+1,:),theta(k+1,:),R0,kappa,N,M,A);
  end
end

%% Figuras
figure(1)
hold on; grid on; axis equal
plot(r)
plot(real(r(n,:)),imag(r(n,:)),'go','linewidth',4)



%% Animacion
x = real(r); y = imag(r);
axs = max(max(abs([x,y])));
figura = figure(6);
set(gcf, 'Position',  [0, 0, 1000, 1000])
axis([-axs-0.5 axs+.5 -axs-0.5 axs+0.5])
hold on; grid on;

for k = 1:3:n
  cla(gca(figura));
  plot(x(k,:),y(k,:),'go','linewidth',4)
  text(3,3,num2str(round(tspan(k),1)),'fontsize',14)
  text(4,3,"seg",'fontsize',14)
  pause(0.05)
  
end

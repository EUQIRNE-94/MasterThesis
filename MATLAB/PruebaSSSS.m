%% Prueba control Sepulchre

% Limpieza de variables alocadas previamente
clear
close all
clc

% Variables Fijas
w0 = 0.5;
N = 6;
M = N;
R0 = 0;
kappa = w0;
controlSep = 5;
rX = 10;
rY = 10;
i = N;
gains = [10,0.1,0.1,0.5,0.1,0.5];
ang_f = (360/i)/2;

% Tiempo
tf = 150;
dt = 0.05;
rev = 2;
tspan = 0:dt:tf;
n = length(tspan);
tcambio = [50,75,125];
ncambio = tcambio/dt;

A = ones(N);

%% Sistema
% Condiciones iniciales sistema Sepulchre
r0 = (rX*rand(1,N)-(rX/2)) + 1i*(rY*rand(1,N)-(rY/2));
theta0 = 2*pi*rand(1,N);

% Condiciones iniciales de uniciclos
x0 = real(r0) + (0.1*real(r0).*rand(1,N) - real(r0)*0.05);
y0 = imag(r0) + (0.1*imag(r0).*rand(1,N) - imag(r0)*0.05);
xi0 = theta0 + (0.1*theta0.*rand(1,N) - theta0*0.05);

%   Prealocacion de variables
% Sepulchre
r = zeros(n,N); theta = zeros(n,N); u = zeros(n,N); c = zeros(n,N);
u_2 = zeros(n,N);
R = zeros(n,1);

% Uniciclo
x = zeros(n,i); y = zeros(n,i); xi = zeros(n,i);                           %
Xr = zeros(n,i); Yr = zeros(n,i); Xir = zeros(n,i);                        %
v = zeros(n,i); w = zeros(n,i);                                            %
err_x = zeros(n,i); err_y = zeros(n,i); err_xi = zeros(n,i);               %

%% Integracion del sistema
%%% Variables iniciales Sepulchre
r(1,:) = r0; theta(1,:) = theta0;

%%% Variables iniciales del uniciclo
x(1,:) = x0; y(1,:) = y0; xi(1,:) = xi0;

for k = 1:n-1
  % Reset de formacion
  if k == ncambio(3)+1
    r(k,:) = x(k,:) + y(k,:)*1i;
    theta(k,:) = xi(k,:);
  end
  
  % Rutas de Sepulchre
  u(k,:) = SteeringControl(w0,r(k,:),theta(k,:),R0,kappa,N,M,controlSep);
  [r(k+1,:),theta(k+1,:)] = ParticlesPlane(r(k,:),theta(k,:),u(k,:),dt);
  
  % Referencias de los uniciclos
  if k <= ncambio(1)
    Xr(k,:) = real(r(k,:));
    Yr(k,:) = imag(r(k,:));
    Xir(k,:) = theta(k,:);
  else
    if k > ncambio(1) && k <= ncambio(2)
%       [Xr(k,:),Yr(k,:),Xir(k,:)] = LineForm(w0,theta(k,:),R0,1 ...
%         ,ang_f,i);
        [Xr(k,:),Yr(k,:),Xir(k,:)] = CircleForm(r(k,:),theta(k,:),R0 ...
          ,1,0.5,i);
    else
      if k > ncambio(2)&& k <= ncambio(3)
%         [Xr(k,:),Yr(k,:),Xir(k,:)] = PointForm(r(k,:),theta(k,:),R0 ...
%           ,1,150,0.25,i);
        [Xr(k,:),Yr(k,:),Xir(k,:)] = CircleForm(r(k,:),theta(k,:),R0 ...
          ,1,0.5,i);
      else
        Xr(k,:) = real(r(k,:));
        Yr(k,:) = imag(r(k,:));
        Xir(k,:) = theta(k,:);
      end
    end
  end
  
  % Obtener errores
  err_xi(k,:) = Xir(k,:) - xi(k,:);
  err_x(k,:) =  cos(xi(k,:)).*(Xr(k,:) - x(k,:))...
    + sin(xi(k,:)).*(Yr(k,:) - y(k,:));
  err_y(k,:) = -sin(xi(k,:)).*(Xr(k,:) - x(k,:))...
    + cos(xi(k,:)).*(Yr(k,:) - y(k,:));
  
  % Obtener velocidades de uniciclos e integrar uniciclo
  [v(k,:),w(k,:)] = TrackingControl(etheta(theta(k,:)),u(k,:)...
    ,err_xi(k,:),err_x(k,:),err_y(k,:),gains);
  [x(k+1,:),y(k+1,:),xi(k+1,:)] = Uni_Mod(x(k,:),y(k,:),xi(k,:)...
    ,v(k,:),w(k,:),dt);
  
  if k == n-1
    % Errores de ultima iteracion
    Xr(k+1,:) = real(r(k+1,:));
    Yr(k+1,:) = imag(r(k+1,:));
    Xir(k+1,:) = theta(k+1,:);
  
    err_xi(k+1,:) = Xir(k+1,:) - xi(k+1,:);
    err_x(k+1,:) =  cos(xi(k+1,:)).*(Xr(k+1,:) - x(k+1,:))...
      + sin(xi(k+1,:)).*(Yr(k+1,:) - y(k+1,:));
    err_y(k+1,:) = -sin(xi(k+1,:)).*(Xr(k+1,:) - x(k+1,:))...
      + cos(xi(k+1,:)).*(Yr(k+1,:) - y(k+1,:));
    
    % Control de ultima iteracion
    u(k+1,:) = SteeringControl(w0,r(k+1,:),theta(k+1,:),R0,kappa,N,M,controlSep);
    [v(k+1,:),w(k+1,:)] = TrackingControl(etheta(theta(k+1,:)),u(k+1,:)...
    ,err_xi(k+1,:),err_x(k+1,:),err_y(k+1,:),gains);
    
  end
end

%% Figuras
axs = max(max(abs([x,y])));

figure(1)
set(gcf, 'Position',  [0, 0, 1000, 1000])
set(gca,'FontSize',16)
hold on; grid on;axis equal
axis([-axs-0.5 axs+.5 -axs-0.5 axs+0.5])
plot(Xr(1:n,:),Yr(1:n,:),'k','linewidth',4)
plot(x(1:n,:),y(1:n,:),'g--','linewidth',1)
for j = 1:N
  UnicycleFigure(x(n,j),y(n,j),xi(n,j),0.2,'blue');
end
plot(Xr(n,:),Yr(n,:),'go','Markersize',10,'linewidth',4)
xlabel('X','fontsize',20)
ylabel('Y','fontsize',20)

% Errores
figure(2)
set(gcf, 'Position',  [0, 0, 1000, 1000])

subplot(3,1,1)
set(gca,'FontSize',20)
title('Errors')
hold on; grid on
plot(tspan,err_x,'linewidth',2)
xlabel('Time [t]','fontsize',22)
ylabel('Error X','fontsize',22)
legend({'e1','e2','e3','e4','e5','e6'},'Orientation','horizontal'...
  ,'location','south','fontsize',14)

subplot(3,1,2)
hold on; grid on
set(gca,'FontSize',20)
plot(tspan,err_y,'linewidth',2)
xlabel('Time [t]','fontsize',22)
ylabel('Error Y','fontsize',22)
legend({'e1','e2','e3','e4','e5','e6'},'Orientation','horizontal'...
  ,'location','south','fontsize',14)

subplot(3,1,3)
hold on; grid on
set(gca,'FontSize',20)
plot(tspan,err_xi,'linewidth',2)
xlabel('Time [t]','fontsize',22)
ylabel('Error \xi','fontsize',22)
legend({'e1','e2','e3','e4','e5','e6'},'Orientation','horizontal'...
  ,'location','south','fontsize',14)

% Entradas de control
figure(3)
set(gcf, 'Position',  [0, 0, 1000, 1000])
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

%% Animacion
figura = figure(6);
set(gcf, 'Position',  [0, 0, 1000, 1000])
axis([-axs-0.5 axs+.5 -axs-0.5 axs+0.5])
hold on; grid on;

for k = 1:2:n
  cla(gca(figura));
  for j = 1:i
    UnicycleFigure(x(k,j),y(k,j),xi(k,j),0.2,'blue');
    text(x(k,j)+.25,y(k,j)+.25,num2str(j))
  end
  plot(Xr(k,:),Yr(k,:),'go','linewidth',4)
  plot(real(r(k,:)),imag(r(k,:)),'r*','linewidth',3)
  text(3,3,num2str(round(tspan(k),1)),'fontsize',14)
  text(4,3,"seg",'fontsize',14)
  pause(0.05)
  
end
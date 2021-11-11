%% Partículas con Dinámica de Oscilador Acoplado
%           SIN CONTROL DE GRADIENTE - ODE Matlab

% Limpieza de variables alocadas previamente
clear
close all
clc

N = 3;            % # de particulas
x_ini = 4;
y_ini = 4;

% Condiciones iniciales
c_ini = zeros(N,2);
for i = 1:N
  c_ini(i,1) = x_ini*rand() + y_ini*rand()*1i;
  c_ini(i,2) = 2*pi*rand();
end

% Entrada de control
w0 = 1;
u_k = w0;
tspan = 0:(2*pi/w0)/30:(2*pi/w0);


% Método de integración
t = zeros(length(tspan),1);
y_t = zeros(length(tspan),2,N);
for i = 1:N
  [t(:,i),y_t(:,:,i)] = ode45(@(t,x)OsAc(t,x,u_k),tspan,c_ini(i,:));
end

% Obtener coordenadas
x = zeros(size(y_t,1),size(y_t,3));
y = zeros(size(y_t,1),size(y_t,3));
theta = zeros(size(y_t,1),size(y_t,3));
r_k = zeros(size(y_t,1),size(y_t,3));           % Estado 1
d_r = zeros(size(y_t,1),size(y_t,3));           % Derivada estado 1
c_k = zeros(size(y_t,1),size(y_t,3));

for i = 1:N
  x(:,i) = real(y_t(:,1,i));
  y(:,i) = imag(y_t(:,1,i));
  theta(:,i) = real(y_t(:,2,i));
  r_k(:,i) = y_t(:,1,i);
  d_r(:,i) = cos(theta(:,i)) + 1i*sin(theta(:,i));
end

for i = 1:N
  c(:,i) = ((1/w0)*(d_r(:,i)))*1i;
  ck_r(:,i) = real(r_k(:,i)) + real(c(:,i));
  ck_i(:,i) = imag(r_k(:,i)) - imag(c(:,i));
end

for i = 1:N
  for j = 1:length(ck_r)
    c_k(j,i) = ck_r(j,i) + 1i*ck_i(j,i);
  end
end

R = (1/N)*sum(r_k,2);
p_theta = (1/N)*sum(d_r,2);

U = zeros(length(p_theta),1);
for i = 1:length(p_theta)
  U(i,1) = (N/2)*(norm(p_theta(i)))^2;
end

% simulacion(t,y_t,R,c_k)


%% Funcion del modelo
% Función del pendulo invertido en un carro con un resorte
  function dx = OsAc(t,x,u_k)
    dx(1) = cos(x(2)) + 1i*sin(x(2));
    dx(2) = u_k;
    dx = dx';   
  end
  
% Simulacion de las particulas
function simulacion(t,y_t,R,c_k)

x = zeros(size(y_t,1),size(y_t,3));
y = zeros(size(y_t,1),size(y_t,3));

for i = 1:size(y_t,3)
  x(:,i) = real(y_t(:,1,i));
  y(:,i) = imag(y_t(:,1,i));
end

x_min = min(min(x));
x_max = max(max(x));
y_min = min(min(y));
y_max = max(max(y));

% str0 = string(strcat('t = ',num2str(round(t(:,1),4)), ' s'));

fig = figure(100);
set(gcf, 'Position', get(0, 'Screensize'));
ax = [x_min-1 x_max+1 y_min-1 y_max+1];
axis (ax)
axis equal
hold on
grid on

n = length(t);

ckx = real(c_k);
cky = imag(c_k);
for i = 1:n
    cla(gca(fig));
    plot([x_min-1,x_min-1,x_max+1,x_max+1],[y_min-1,y_max+1,y_max+1,y_min-1])
    plot(x(i,:),y(i,:),'ob','linewidth',3)
    plot(real(R(i)),imag(R(i)),'*r','linewidth',1)
    plot(ckx(i,:),cky(i,:),'*m','linewidth',1)
    drawnow
    pause(.1)
end

hold on
for i = 1:size(x,2)
  plot([x(1,i),ckx(1,i)],[y(1,i),cky(1,i)],'k')
  plot(x(:,i),y(:,i),'g','linewidth',2)
axis equal

end

% close all


end


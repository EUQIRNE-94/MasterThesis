%% Partícula con Dinamica Oscilatoria
%           PARTICULA UNICA

% Limpieza de variables alocadas previamente
clear
close all
% clc

% Constantes de los modelos
w0 = 1;
rev = 1;
dt = rev*(2*pi/w0)/30;
tspan = 0:dt:rev*(2*pi/w0);

% Entrada de control
u_k = w0;

% Integración de 1 particula
c_ini = [(2 + 2*1i) 0];                % Condiciones iniciales
% c_ini = [(4*rand() + 4*rand()*1i) 2*pi*rand()];

% Método de integración
[t,y_t] = ode45(@(t,x)OsAc(t,x,u_k),tspan,c_ini);

% Obtener coordenadas
r_k = y_t(:,1);
theta = real(y_t(:,2));

x = real(r_k);
y = imag(r_k);
e_thetak = cos(theta) + 1i*sin(theta);
c = ((1/w0)*(e_thetak))*1i;
cp = r_k + ((1/w0)*(e_thetak))*1i;
% ck = [real(r_k) + real(c), imag(r_k) - imag(c)];
ck = [real(r_k)- (sin(theta)/w0), imag(r_k) - (cos(theta)/w0)];

figure(1)
plot(real(r_k),imag(r_k));hold on
plot(ck(:,1),ck(:,2),'r*')
grid on
axis equal

%% Simulacion
n = length(r_k);
fig = figure(2);
ax = [ min(min([x,ck(:,1)])), max(max([x,ck(:,1)])), min(min([y,ck(:,2)])), max(max([y,ck(:,2)])) ];
axis (ax)
axis equal
hold on; grid on
for i = 1:n
    cla(gca(fig));
    plot(x(i),y(i),'ob','linewidth',10)
    plot(real(cp(i,1)),imag(cp(i)),'g*','linewidth',2)
    plot(ck(i,1),ck(i,2),'r*','linewidth',2)
    legend({'Particula','Centro-Paper','Centro-Mod'})
    drawnow
    pause(0.1)
end

hold on
axis equal
plot(x,y)

%% Funcion del modelo
% Función del pendulo invertido en un carro con un resorte
  function dx = OsAc(t,x,u_k)
    dx(1) = cos(x(2)) + 1i*sin(x(2));
    dx(2) = u_k;
    
    dx = dx';
  end
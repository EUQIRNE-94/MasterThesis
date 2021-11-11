clear
clc
close all
%%Variables de referencia
xr = 0;         %Punto de inicio en X
yr = 0;         %Punto de inicio en Y
thetar = 0;    %Orientacion de inicio en rad
A = 5;          %Amplitud de la vuelta
f = .05;        %Frecuencia del giro
dt = .01;       %Diferencial del tiempo
t = 0;          %Tiempo de inicio
tfin = 20;     %Tiempo de fin del ciclo

%Variables de inicio en el bucle de movimiento
theta = thetar;
x = xr;
y = yr;

%Constantes de los errores
kx = .5;
ky = 150;
ktheta = 1;
k = 1;

%%Movimiento del robot
i = 1;

figura = figure(1);
axis([-10 10 -10 10])
hold on
grid on

while (t < tfin)
    %%Variables de referencia
    xr = A*cos(2*pi*f*t);
    yr = A*sin(2*pi*f*t);
    %Primera derivada
    dx = -2*A*pi*f*sin(2*pi*f*t);
    dy = 2*A*pi*f*cos(2*pi*f*t);
    %Segunda derivada
    dxx = - ((2*pi*f)^2)*A*cos(2*pi*f*t);
    dyy = - ((2*pi*f)^2)*A*sin(2*pi*f*t);
    
    thetar = atan2(dy,dx);
    
    %%Error con respecto al punto anterior
    errX = cos(theta)*(xr - x) + sin(theta)*(yr - y);
    errY = -sin(theta)*(xr - x) + cos(theta)*(yr - y);
    errTheta = thetar - theta;
    
    %%Velocidades referencia
    vr = sqrt((dx^2) + (dy^2));
    wr = (dyy*dx - dxx*dy)/((dx^2) + (dy^2));
    
    %%Velocidades de salida a los motores
    alpha = sqrt(k^2 + (errX^2) + (errY^2));
    %% Salidas que utilizare para enviar al compensador del robot.
    v = vr*cos(errTheta) + kx*errX;
    if (errTheta == 0)
        w = wr + ktheta*errTheta + vr*(k/alpha)*ky*errY;
    else
        w = wr + ktheta*errTheta + vr*(k/alpha)*(sin(errTheta)/errTheta)*ky*errY;
    end
    
    if abs(w) > 1e-6
        a = v/(.5*w)*sin(.5*w*dt);
    else
        a= v*dt;
    end
    %%
    %%Puntos
    x = x + a*cos(theta + 2*w*.5*dt);
    y = y + a*sin(theta + 2*w*.5*dt);
    theta = theta + w*dt;
    
    t = t + dt;
    i = i + 1;
    
    cla(gca(figura));
    mv(xr,yr,thetar,.65,'green');
    mv(x,y,theta,.5,'red');
    
    text(x,y,'Robot 1')
    text(9,9,num2str(t));
    drawnow
end
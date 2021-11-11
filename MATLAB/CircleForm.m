% Funcion para generar las velocidades del uniciclo
% Variables de entrada
%       r      -> Posicion de las particulas
%       theta  -> Fase de las particulas
%       R      -> Centro de giro de la formacion simetrica
%       part   -> Particula como lider virtual
%       radius -> Radio del circulo a realizar
%       i      -> Cantidad de uniciclos
%
% Variables de salida
%       Xr  -> Referencia en 'x'
%       Yr  -> Referencia en 'y'
%       Xir -> Referencia en orientacion 'xi'
%
% La funcion permite la entrada de vectores para la integracion
function [Xr,Yr,Xir] = CircleForm(r,theta,R,part,radius,i)
% Circulo alrededor de una particula especifica
a = radius;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obtener acomodo de las particulas
deltas = theta - theta(part);
deltas = round(deltas/(2*pi/i))-i;
deltas = mod(deltas,-i);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

deltaX = zeros(1,i); deltaY = zeros(1,i); deltaxi = zeros(1,i);
ang = zeros(1,i);
angulos = mod(theta,2*pi); % angulo del robot

for k = 1:i
  deltaX(1,k) = real(r(part)) + a*cos(theta(part) + deltas(k)*(2*pi/i));
  deltaY(1,k) = imag(r(part)) + a*sin(theta(part) + deltas(k)*(2*pi/i));
  
  % Angulo de referencia de cada particula perpendicular a su radio con
  % respecto al centro de giro
  ang(k) = atan2(deltaY(k)-imag(R),deltaX(k)-real(R));
  if ang(k) < 0
    ang(k) = ang(k) + 2*pi;
  end
  ang(k) = ang(k) + pi/2;
  ang(k) = mod(ang(k),2*pi);
  deltaxi(k) = ang(k) -  angulos(k);
  if rad2deg(deltaxi(k)) < 0
    deltaxi(k) = deltaxi(k) + 2*pi;
  end
  
Xr = deltaX;
Yr = deltaY;
Xir = theta + deltaxi;
end
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
function [Xr,Yr,Xir] = PointForm(r,theta,R,part,angulo,radio,i)
% Formacion en punta de una particula especifica
a = zeros(1,i);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obtener acomodo de las particulas
deltas = theta - theta(part);
deltas = round(deltas/(2*pi/i))-i;
deltas = mod(deltas,-i);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for o = 1:i
  kk = ceil(abs(deltas(o))/2);
  a(1,o) = radio*kk;
end

deltaX = zeros(1,i); deltaY = zeros(1,i); deltatheta = zeros(1,i);
beta = deg2rad(angulo);

ang = zeros(1,i);
angulos = mod(theta,2*pi); % angulo del robot

for o = 1:i
  deltaX(1,o) = real(r(part)) + a(o)*cos(theta(part) + ((-1)^(-deltas(o)))*beta );
  deltaY(1,o) = imag(r(part)) + a(o)*sin(theta(part) + ((-1)^(-deltas(o)))*beta );
  
  % Angulo de referencia de cada particula perpendicular a su radio con
  % respecto al centro de giro
  ang(o) = atan2(deltaY(o)-imag(R),deltaX(o)-real(R));
  if ang(o) < 0
    ang(o) = ang(o) + 2*pi;
  end
  ang(o) = ang(o) + pi/2;
  ang = mod(ang,2*pi);
  deltatheta(o) = ang(o) -  angulos(o);
  if rad2deg(deltatheta(o)) < 0
    deltatheta(o) = deltatheta(o) + 2*pi;
  end
end

% deltatheta(part) = 0;

Xr = deltaX;
Yr = deltaY;
Xir = theta + deltatheta;
end
% Funcion para generar las velocidades del uniciclo
% Variables de entrada
%       w0        -> Frecuencia natural
%       r         -> Posicion de las particulas
%       theta     -> Fase de las particulas
%       R         -> Centro de giro de la formacion simetrica
%       part      -> Particula como lider virtual
%       delta_ang -> Separacion angular entre uniciclos
%       i         -> Cantidad de uniciclos
%
% Variables de salida
%       Xr  -> Referencia en 'x'
%       Yr  -> Referencia en 'y'
%       Xir -> Referencia en orientacion 'xi'
%
% La funcion permite la entrada de vectores para la integracion
function [Xr,Yr,Xir] = LineForm(w0,r,theta,R,part,delta_ang,i)
% Formacion en seguimiento a una particula especifica
deltagamma = deg2rad((360/i) - delta_ang);

deltaX = zeros(1,i); deltaY = zeros(1,i); deltatheta = zeros(1,i);
rho = 1/w0;

%% Orden de las particulas
deltas = theta - theta(part);
deltas = round(deltas/(2*pi/i))-i;
deltas = mod(deltas,-i);
%%

for k = 1:i
  deltatheta(1,k) = theta(k) - (deltas(k))*deltagamma;
  deltaX(1,k) = rho*cos(-pi/2 + deltatheta(1,k) );
  deltaY(1,k) = rho*sin(-pi/2 + deltatheta(1,k) );
end

Xr = real(R) + deltaX;
Yr = imag(R) + deltaY;
Xr(1) = real(r(part));
Yr(1) = imag(r(part));
Xir = deltatheta;

end
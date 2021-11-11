% Funcion para integrar el modelo del uniciclo en el tiempo
% Variables de entrada
%       x_in  -> Posicion 'x' del uniciclo en el instante 't'
%       y_in  -> Posicion 'y' del uniciclo en el instante 't'
%       xi_in -> Orientacion del uniciclo en el instante 't'
%       v     -> Velocidad lineal en el instante 't'
%       w     -> Velocidad angular en el instannte 't'
%       dt    -> Paso de integracion
%
% Variables de salida
%       x  -> Posicion 'x' del uniciclo en el instante 't + dt'
%       y  -> Posicion 'y' del uniciclo en el instante 't + dt'
%       xi -> Orientacion del uniciclo en el instante 't + dt'
% 
% La funcion permite la entrada de vectores para la integracion
function [x,y,xi] = Uni_Mod(x_in,y_in,xi_in,v,w,dt)
  x = x_in + v.*cos(xi_in)*dt;
  y = y_in + v.*sin(xi_in)*dt;
  xi = xi_in + w*dt;
end
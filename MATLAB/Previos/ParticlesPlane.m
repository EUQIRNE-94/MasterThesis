% Funcion para integrar el modelo de particulas en el plano complejo en el
% tiempo
% Variables de entrada
%       r_in  -> Posicion 'x+yi' de la particula en el instante 't'
%       theta_in  -> Orientacion de la particula en el instante 't'
%       u_in -> Entrada de control de la particula en el instante 't'
%       dt    -> Paso de integracion
%
% Variables de salida
%       r  -> Posicion 'x+yi' de la particula en el instante 't + dt'
%       theta  -> Orientacion de la particula en el instante 't + dt'
% 
% La funcion permite la entrada de vectores para la integracion
function [r,theta] = ParticlesPlane(r_in,theta_in,u_in,dt)
  r = r_in + etheta(theta_in)*dt;
  theta = theta_in + u_in*dt;
end
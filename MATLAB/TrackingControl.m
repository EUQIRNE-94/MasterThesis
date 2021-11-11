% Funcion para generar las velocidades del uniciclo
% Variables de entrada
%       etheta    -> Derivada de la posicion => dr/dt
%       dtheta    -> Derivada de la orientación => SteeringControl
%       err_xi    -> Error entre referencia 'theta' de orientacion y del
%                    uniciclo
%       err_x     -> Error entre referencia 'x' del posicion y del uniciclo
%       err_y     -> Error entre referencia 'y' del posicion y del uniciclo
%       Ks        -> Ganancias del sistema [Cx,Cxk,Cxi,Cy,Cyk,K]
%
% Variables de salida
%       v  -> Velocidad lineal del uniciclo
%       w  -> Velocidad angular del uniciclo
% 
% La funcion permite la entrada de vectores para la integracion
function [v,w] = TrackingControl(V0,etheta,dtheta,err_xi,err_x,err_y,Ks)

% Ganancias de control
Cx = Ks(1); Cxk = Ks(2); Ctheta = Ks(3); Cy = Ks(4); Cyk = Ks(5); K = Ks(6);

% Cantidad de uniciclos
i = length(etheta);

% Prealocacion de variables
norm_v = zeros(1,i); copX = zeros(1,i); copY = zeros(1,i); k1 = zeros(1,i);

for k = 1:i
  % Velocidad de referencia
  norm_v(1,k) = norm(etheta(k));
  
  % Control de giro
  if err_xi(i) <= 1e-6
    k1(1,k) = 1;
  else
    k1(1,k) = sin(err_xi(k))./err_xi(k);
  end
  
  % Error de acoplamiento
  for j = 1:i
    copX(1,k) = copX(1,k) + (err_x(k) - err_x(j));
    copY(1,k) = copY(1,k) + (err_y(k) - err_y(j));
  end
end

% v = norm_v.*cos(err_xi) + Cx*(err_x + Cxk*copX);
v = norm_v + Cx*(err_x + Cxk*copX);

for k = 1:i
  if abs(v(k)) > 2*V0
    v(k) = 2*V0;
  end
end

alpha = sqrt(K^2 + err_x.^2 + err_y.^2);
w = dtheta + Ctheta.*err_xi + (norm_v.*(k1)).*(K./alpha).*Cy.*(err_y + Cyk*copY);
end
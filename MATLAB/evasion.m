% Variables de entrada
%       x         -> Posición X del objeto
%       y         -> Posicion Y del objeto
%       d1        -> Distancia radial del objeto
%       d2        -> Distancia radial de protección
%       K         -> Ganancia para la función de tangente hiperbólica
%       
% Variables de salida
%       rep  -> Factor de repulsión en forma de vector
%       d    -> Matriz de distancias entre elementos
%
% La funcion calcula el valor de repulsion entre los elementos por medio de
% la funcion:
%
%           rep = 1 - tanh(K*(d2-d)/(d2-d1))

function [rep,d] = evasion(x,y,d1,d2,K)
  
  N = length(x);
  rep = zeros(N,N);
  M = zeros(N);
  
  for k = 1:N
    for j = 1:N
      M(k,j) = norm([x(k)-x(j),y(k)-y(j)]);
      if M(k,j) < d2 && M(k,j) > 0
        rep(k,j) = tanh(K*( (d2-M(k,j))/(d2-d1) ));
      end
    end
  end
  
  rep = 1 - rep;
  rep = tril(ones(N)).*rep + triu(ones(N));
  rep = min(rep,[],2);
  d = M;
end
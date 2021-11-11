% Variables de entrada
%       vec         -> Posición X del objeto
%       A         -> Posicion Y del objeto
%
% Variables de salida
%       M  -> Factor de repulsión en forma de vector
%
% La funcion genera una matriz de adyacencia limitada para realizar ingreso
% y egreso de elementos a la formación.
function M = matrFor(vec,A)
  for k = 1:length(vec)
    A(vec(k),:) = 0;
    A(:,vec(k)) = 0;
  end
  
  M = A;
end
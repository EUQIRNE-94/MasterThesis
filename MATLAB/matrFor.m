% Variables de entrada
%       vec -> particulas fuera de la formacion
%       A   -> Matriz de adyacencia de comunicación
%
% Variables de salida
%       M  -> Matriz de salida de formacion
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
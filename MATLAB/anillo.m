function [A,G] = anillo(N)
% Crear vectores de nodos padre, hijo y peso
np = 1:N;
nh = 2:N+1;
p = ones(1,N);

% Completar para que sea una matriz cuadrada con el elemento NxN con peso 0
nh(N) = 1;

% Crear matriz de adyacencia
As = sparse(np,nh,p);
A = full(As);

if nargout == 2
  G = digraph(A');
end

end
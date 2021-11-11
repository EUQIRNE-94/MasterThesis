% Funcion de posicion en el plano complejo de e^(theta*i)_k
function res = etheta(theta,V0)
  res = V0*(cos(theta) + 1i*sin(theta));
end
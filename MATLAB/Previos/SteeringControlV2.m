% Funcion para generar el control de direccion de Sepulchre
% Variables de entrada
%       w0        -> Frecuencia natural
%       r -> Posicion de la particula en el plano
%       theta     -> Fase de las particulas
%       R         -> Centro de giro de la formacion simetrica
%       kappa      -> Ganancia de sistema
%       N -> Cantidad de partículas
%       M         -> Cantidad de subgrupos
%       A -> Matriz de adyacencia
%
% Variables de salida
%       u  -> Control de direccion
%
% La funcion permite la entrada de vectores para la integracion
function [u,U1,U2,U3] = SteeringControlV2(w0,r,theta,R,kappa,N,M,A)
  U1 = u1(V0,w0,r,R,theta,kappa);
  U2 = u2(theta,N,kappa,A);
  U3 = u3(w0,theta,N,M,A);
  u = U1 + U2 + U3;
  
  % Funciones para control de particulas en el plano
  function res = u1(V0,w0,r,R,theta,kappa)
    rt = r-R;
    rp = etheta(theta);
    dp = real(rt).*real(rp) + imag(rt).*imag(rp);
    res = w0*(1 + kappa*dp);
  end
  function res = u2(theta,N,kappa,A)
    s_theta = sin(theta);
    c_theta = cos(theta);
    res = diag(s_theta)*A*c_theta' - diag(c_theta)*A*s_theta';
    res = (kappa/N)*res';
  end
  function res = u3(w0,theta,N,M,A)
    r3 = zeros(N,M);
    km = zeros(M,1);
    km(:,1) = 0.1*w0;         % Reducir ganancia para mejores resultados
    km(M,1) = -km(M,1);
    for m = 1:M
      s_theta = sin(m*theta);
      c_theta = cos(m*theta);
      r3(:,m) = (km(m)/m)*(diag(s_theta)*A*c_theta' - diag(c_theta)*A*s_theta');
    end
    res = sum(r3,2)';
  end
end
% Variables de entrada
%       w0        -> Frecuencia natural
%       r -> Posicion de la particula en el plano
%       theta     -> Fase de las particulas
%       R         -> Centro de giro de la formacion simetrica
%       kappa      -> Ganancia de sistema (kappa > 0)
%       N -> Cantidad de partículas
%       M         -> Cantidad de subgrupos
%       A -> Matriz de adyacencia
%       ConNum -> Tipo de control
%         0 -- Sin control
%         1 -- Sin control + w0
%         2 -- Acoplamiento Senoidal
%         3 -- Acoplamiento Senoidal + w0
%         4 -- Arreglo Espacial
%         5 -- Arreglo Espacial + Acoplamiento Senoidal
%         6 -- Arreglo Espacial + Acoplamiento Senoidal + Acoplamiento
%              Senoidal para Fomarcion Simetrica
%
% Variables de salida
%       u  -> Control de direccion
%
% La funcion permite la entrada de vectores para la integracion
function u = SteeringControl(V0,w0,r,theta,R,kappa,N,M,ConNum)
  switch ConNum
    % Ningun control aplicado
    case 0
      U1 = 0;
      U2 = 0;
      U3 = 0;
      
    % Entrada de control es frecuencia natural
    case 1
      U1 = w0;
      U2 = 0;
      U3 = 0;
      
    % Control es acoplamiento senoidal solamente
    case 2
      U1 = 0;
      U2 = -u2(theta,N,kappa);
      U3 = 0;
      
    % Control es acoplamiento senoidal y fecuencia natural
    case 3
      U1 = w0;
      U2 = -u2(theta,N,kappa);
      U3 = 0;
    
      % Control es formacion espacial
    case 4
      U1 = u1(V0,w0,r,R,theta,kappa);
      U2 = 0;
      U3 = 0;
      
    % Control es acoplamiento senoidal y control espacial
    case 5
      U1 = u1(V0,w0,r,R,theta,kappa);
      U2 = u2(theta,N,kappa);
      U3 = 0;
      
    % Control es acoplamiento senoidal, control espacial y control de
    % formación simetrica
    case 6
      U1 = u1(V0,w0,r,R,theta,kappa);
      U2 = u2(theta,N,kappa);
      U3 = u3(w0,theta,N,M);
      
    otherwise
      msg = 'Control no esta enlistado \n Valores entre 0 - 6';
      error(msg)
  end
  u = U1 + U2 + U3;
  
  % Funciones para control de particulas en el plano
  function res = u1(v0,w0,r,R,theta,kappa)
    rt = r-R;
    rp = etheta(theta,v0);
    dp = real(rt).*real(rp) + imag(rt).*imag(rp);
    res = w0*(1 + kappa*dp);
  end
  function res = u2(theta,N,kappa)
    s_theta = sin(theta);
    c_theta = cos(theta);
    A = ones(N);
    res = diag(s_theta)*A*c_theta' - diag(c_theta)*A*s_theta';
    res = (kappa/N)*res';
  end
  function res = u3(w0,theta,N,M)
    r3 = zeros(N,M);
    km = zeros(M,1);
    km(:,1) = 0.1*w0;         % Reducir ganancia para mejores resultados
    km(M,1) = -km(M,1);
    A = ones(N);
    for m = 1:M
      s_theta = sin(m*theta);
      c_theta = cos(m*theta);
      r3(:,m) = (km(m)/m)*(diag(s_theta)*A*c_theta' - diag(c_theta)*A*s_theta');
    end
    res = sum(r3,2)';
  end
end
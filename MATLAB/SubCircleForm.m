function [Xr,Yr,Xir] = SubCircleForm(r,theta,R,SubG,radius,i)
% Circulo alrededor de una particula especifica
a = radius;
%% Orden de las particulas 

if SubG >= 2 && SubG < i
  %%%% para 2 <= SubG < i
  particulas  = zeros(1,SubG);
  Lid = ones(1,SubG);
  lids = ones(1,i);
  
  kk = 1;
  for k = 1:i
    particulas(kk) = particulas(kk) + 1;
    kk = kk+1;
    if kk == SubG+1
      kk = 1;
    end
  end
  
  angs = ones(1,i)*(2*pi/particulas(1));
  for k = 2:SubG
    Lid(k) = Lid(k-1) + particulas(k-1);
  end

  deltas = theta - theta(1);
  deltas = round(deltas/(2*pi/i))-i;
  deltas = mod(deltas,-i);

  for j = 1:SubG-1
    index = find(abs(deltas) == Lid(j+1)-1);
    
    for k = 1:i
      if abs(deltas(k)) >= (Lid(j) - 1) + particulas(j) && abs(deltas(k)) < (Lid(j+1)-1)+particulas(j+1)
        deltas(k) = deltas(k) + Lid(j+1) - 1;
        lids(k) = index(1);
        angs(k) = 2*pi/particulas(j+1);
      end
    end
    
  end
  
else
    deltas = theta - theta(1);
    deltas = round(deltas/(2*pi/i))-i;
    deltas = mod(deltas,-i);
    lids = ones(1,i);
  angs = ones(1,i)*(2*pi/i);
end


%% Generar figura

deltaX = zeros(1,i); deltaY = zeros(1,i); deltaxi = zeros(1,i);
ang = zeros(1,i);
angulos = mod(theta,2*pi); % angulo del robot

for k = 1:i
  deltaX(1,k) = real(r(lids(k))) + a*cos(theta(lids(k)) + deltas(k)*(angs(k)));
  deltaY(1,k) = imag(r(lids(k))) + a*sin(theta(lids(k)) + deltas(k)*(angs(k)));
  
  % Angulo de referencia de cada particula perpendicular a su radio con
  % respecto al centro de giro
  ang(k) = atan2(deltaY(k)-imag(R),deltaX(k)-real(R));
  if ang(k) < 0
    ang(k) = ang(k) + 2*pi;
  end
  ang(k) = ang(k) + pi/2;
  ang = mod(ang,2*pi);
  deltaxi(k) = ang(k) -  angulos(k);
  if rad2deg(deltaxi(k)) < 0
    deltaxi(k) = deltaxi(k) + 2*pi;
  end
  
end

Xr = deltaX;
Yr = deltaY;
Xir = theta + deltaxi;


end
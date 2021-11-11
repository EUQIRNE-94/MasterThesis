function [Xr,Yr,Xir] = SubPointForm(r,theta,R,SubG,angulo,radio,i)
% Formacion en punta de una particula especifica
a = zeros(1,i);
%% Orden de las particulas 

if SubG >= 2 && SubG < i
  %%%% para 2 < SubG < i-1
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

  for k = 2:SubG
    Lid(k) = Lid(k-1) + particulas(k-1);
  end

  deltas = theta - theta(1);
  deltas = round(deltas/(2*pi/i))-i;
  deltas = mod(deltas,-i);

  for j = 1:SubG-1
    index = find(abs(deltas) == Lid(j+1)-1);
    for k = 1:i
      if abs(deltas(k)) >= (Lid(j)-1)+particulas(j) && abs(deltas(k)) < (Lid(j+1)-1)+particulas(j+1)
        deltas(k) = deltas(k) + Lid(j+1) - 1;
        lids(k) = index(1);
      end
    end
  end
  
else
    deltas = theta - theta(1);
    deltas = round(deltas/(2*pi/i))-i;
    deltas = mod(deltas,-i);
    lids = ones(1,i);
end

for o = 1:i
  kk = ceil(abs(deltas(o))/2);
  a(1,o) = radio*kk;
end

deltaX = zeros(1,i); deltaY = zeros(1,i); deltatheta = zeros(1,i);
beta = deg2rad(angulo);

ang = zeros(1,i);
angulos = mod(theta,2*pi); % angulo del robot

% pause(0.1)

for o = 1:i
  deltaX(1,o) = real(r(lids(o))) + a(o)*cos(theta(lids(k)) + ((-1)^(-deltas(o)))*beta );
  deltaY(1,o) = imag(r(lids(o))) + a(o)*sin(theta(lids(k)) + ((-1)^(-deltas(o)))*beta );
  
  % Angulo de referencia de cada particula perpendicular a su radio con
  % respecto al centro de giro
  ang(o) = atan2(deltaY(o)-imag(R),deltaX(o)-real(R));
  if ang(o) < 0
    ang(o) = ang(o) + 2*pi;
  end
  ang(o) = ang(o) + pi/2;
  ang = mod(ang,2*pi);
  
  deltatheta(o) = ang(o) -  angulos(o);
  if rad2deg(deltatheta(o)) < 0
    deltatheta(o) = deltatheta(o) + 2*pi;
  end
end
pp = unique(lids);
for k = 1:SubG
  deltatheta(pp(k)) = 0;
end
Xr = deltaX;
Yr = deltaY;
Xir = theta + deltatheta;
end
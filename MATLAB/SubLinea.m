function [Xr,Yr,XIr] = SubLinea(V0,w0,theta,R,SubG,delta_ang,i)
% Formacion en seguimiento a varias particulas especificas
deltagamma = deg2rad((360/i) - delta_ang);

deltaX = zeros(1,i); deltaY = zeros(1,i); deltatheta = zeros(1,i);
rho = V0/w0;


%% Orden de las particulas 

if SubG >= 2 && SubG < i
  %%%% para 2 < SubG < i-1
  particulas  = zeros(1,SubG);
  Lid = ones(1,SubG);

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
    for k = 1:i
      if abs(deltas(k)) >= (Lid(j)-1)+particulas(j) && abs(deltas(k)) < (Lid(j+1)-1)+particulas(j+1)
        deltas(k) = deltas(k) + Lid(j+1) - 1;
      end
    end
  end
else
  deltas = theta - theta(1);
  deltas = round(deltas/(2*pi/i))-i;
  deltas = mod(deltas,-i);
end


%%

for k = 1:i
  deltatheta(1,k) = theta(k) - (deltas(k))*deltagamma;
  deltaX(1,k) = rho*cos(-pi/2 + deltatheta(1,k) );
  deltaY(1,k) = rho*sin(-pi/2 + deltatheta(1,k) );
end

Xr = real(R) + deltaX;
Yr = imag(R) + deltaY;
XIr = deltatheta;

end
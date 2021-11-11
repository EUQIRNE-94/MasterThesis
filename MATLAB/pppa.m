clear
clc
theta = [41.7032,40.1599,36.2024,36.9957,37.7800,40.9291,39.3622,38.5889];
SubG = 3;
N = length(theta);

% Vectores
Lids = zeros(1,SubG);
part = ones(1,SubG)*floor(N/SubG);
ind = zeros(1,SubG) + 1;

lids = ones(1,N);

delta0 = theta - theta(1);
delta0 = round(delta0/(2*pi/N))-N;
delta0 = mod(delta0,-N);
deltas = delta0;


%% Obtener grupos de particulas
kk = 1;
sumP = sum(part);
while sumP ~= N && sumP < N
  if sumP < N
    part(kk) = part(kk) + 1;
  end
  kk = kk + 1;
  sumP = sum(part);
end

%% Obtener Lideres virtuales y posicion del lider en la formacion
for k = 1:SubG-1
  Lids(k+1) = Lids(k) + part(k);
  ind(k+1) = find(abs(delta0) == Lids(k+1));
end

%% Generar vector de posiciones y vector auxiliar de lideres
for j = 1:SubG
  deltap = delta0 + Lids(j);
  for k = 1:N
    if deltap(k) <= 0
      deltas(k) = deltap(k);
      lids(k) = ind(j);
    end
  end
end
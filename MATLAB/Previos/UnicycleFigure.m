% Figura de un pentagono en la posicion (x,y) y orientacion 'theta' con lados
% de valor 'escala' y 'color' especificado.
function pent = UnicycleFigure(x,y,theta,escala,color)
  xp = escala*[-1,1,1.5,1,-1];
  yp = escala*[-1,-1,0,1,1];

  rot = [cos(theta),-sin(theta);
  sin(theta),cos(theta)];

  pos = rot*([xp;yp]) + [x;y]*ones(1,length(xp));

  pent = patch(pos(1,:),pos(2,:),color);
end
function pos = func_iteracionaldt(pos,controller,dt)
v = controller.v;
w = controller.w;


A = v*dt;

pos.x = pos.x + A*cos(pos.theta + .5*w*dt);
pos.y = pos.y + A*sin(pos.theta + .5*w*dt);
pos.theta = pos.theta + dt*w;
end
function ode_sin

 hold on
 for y0 = -pi:1:pi
 [t,y] = ode23s(@sin_ex01, [0 10],y0);
 plot(t,y(:,1),'b','LineWidth',2);
 xlabel('Tiempo','fontsize',14);ylabel('solucion-x','fontsize',4);
 title('Soluciones de dx/dt = sin(x)','fontsize',16);
 set(gcf,'Color', [1 1 1])
 grid on
 end
 hold off
 
 
 function dy = sin_ex01(~,y)
 dy = zeros(1,1);
 dy(1) = sin(y(1));
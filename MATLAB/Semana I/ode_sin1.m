function ode_sin1
    clear all
    close all
    
    hold on
    for y0 = [.1;0]
        [t,y] = ode45(@sin_ex01, [0 1.5],y0);
        figure(1)
        plot(t,y(:,1),'b', t,y(:,2), 'b:', 'LineWidth',2);
        xlabel('Tiempo','fontsize',14);ylabel('Concentracion','fontsize',14);
        title('Soluciones de f(x)','fontsize',16);
        set(gcf,'Color', [1 1 1])
        grid on
        
        figure(2)
        plot(y(:,1),y(:,2),'b','LineWidth',2);
        xlabel('Tiempo','fontsize',14);ylabel('solucion-x','fontsize',4);
        title('Soluciones de f(x)','fontsize',16);
        set(gcf,'Color', [1 1 1])
        grid on
    end
    hold off
 
 
 function dy = sin_ex01(~,y)
    k1 = 20; k2 = 5; k = 1; k3 = 5; k4 = 5; k5 = 2; n = 4;
    dy = zeros(2,1);
    
    dy(1) = k1/(1 + (y(2)/k))^n - k3*y(1) - k5*y(1);
    dy(2) = k2 + k5*y(1) - k4*y(2) - k5*y(1);
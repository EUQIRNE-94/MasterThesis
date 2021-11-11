function sal = mv(x,y,theta,lado,color)
global hax;
A = [-lado     lado/2    lado    lado/2    -lado
     -lado    -lado      0       lado       lado
      0        0         0       0          0
    ];

rot = [
     cos(theta) -sin(theta) 0
     sin(theta)  cos(theta) 0
     0           0          1
    ];

A = rot*A + [x*ones(1,5); y*ones(1,5); 0*ones(1,5)];

if exist('prevpatch')
    set(prevPatch,'Xdata',A(1,:))
    set(prevPatch,'Xdata',A(2,:))
else
    if exist('hax') == 1 && numel(hax) > 0
        sal = patch(A(1,:),A(2,:),color,'Parent',hax);
    else
        sal = patch(A(1,:),A(2,:),color);
    end
end
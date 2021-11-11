% jebiwbñ
clear
clc

d1 = 0.5;
d2 = 0.25;
k = 1;

d = 2:-.001:0;
d = d';

deltad = d1-d2;
deltar = d - d2;

y = tanh(k*((deltad-deltar)/deltad));

plot(d,y)
clear;

t = 0;
x = 0;
u = 0;

[sys,x,str,ts] = mpcFunction(t,x,u,0);
mpcFunction(t,x,u,3)


function state = getNextState(X,u)

%% 最垃圾的方式：线性化后再向前欧拉
% [A_Tk,B_Tk,C_Tk] = getDiscreteMatrix(X,u);
% state = A_Tk*X + B_Tk*u + C_Tk;

%% RK4
T = getParameter('T');
k1 = differentialFunction(X,u);
k2 = differentialFunction(X + T/2*k1,u);
k3 = differentialFunction(X + T/2*k2,u);
k4 = differentialFunction(X + T*k3,u);
state = X + T/6*(k1 + 2*k2 + 2*k3 + k4);

end
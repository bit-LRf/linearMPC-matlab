
function state = getNextState(X,u)
[A_Tk,B_Tk,C_Tk] = getDiscreteMatrix(X,u);
state = A_Tk*X + B_Tk*u + C_Tk;
end
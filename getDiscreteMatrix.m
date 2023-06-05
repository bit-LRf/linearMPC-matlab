% 模型线性化函数，需要提前获得雅克比线性化的表达式
function [A_Tk,B_Tk,C_Tk] = getDiscreteMatrix(X_k,u_k)
% 解算状态矩阵和控制矩阵
x = X_k;
u = u_k;

% 模型基础参数
T = getParameter('T');

% 雅克比线性化模型
X_dot_k = u^3;

DfDX_k = 0;
DfDu_k = 3*u^2;

A_k = DfDX_k;
B_k = DfDu_k;
C_k = X_dot_k - DfDX_k*X_k - DfDu_k*u_k;

% 向前欧拉
n = size(A_k);
A_Tk = eye(n) + T*A_k;
B_Tk = T*B_k;
C_Tk = T*C_k;
% C_Tk = [];

% z变换
% A_Tk = expm(A_k.*T);
% syms tao;
% b = @(tao)expm(A_k.*tao);
% B_Tk = integral(b,0,T,'ArrayValued',true)'*B_k;
% C_Tk = integral(b,0,T,'ArrayValued',true)'*C_k;

end
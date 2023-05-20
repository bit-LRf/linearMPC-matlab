clear;
syms x u;

% 输入非线性表达式、状态量和输入量
map = u^3;
X = x;
U = u;

% 雅克比线性化
A = jacobian(map,X)
B = jacobian(map,U)
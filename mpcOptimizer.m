% lmpc求解器
% 调用自定义函数：
% getDiscreteMatrix
% getNextState
% getParameter

function [X,fval,flag,Y_opt] = mpcOptimizer(X_0,u_n1,Diff_U_eq,Y_ref)
%% 根据参考控制量和当前状态量预测未来状态序列，并将储存平衡点附近的线性化方程。需要X_0和U_eq
Np = getParameter('Np');
Nu = getParameter('Nu');
Nx = getParameter('Nx');

% 得到平衡点控制增量序列
U_n1 = kron(ones(Np,1),u_n1);

% 生成平衡点控制量序列
A_I = zeros(Np,Np);
for i = 1:Np
    for j = 1:Np
        if i>=j
            A_I(i,j) = 1;
        end
    end
end
A_I = kron(A_I,eye(Nu,Nu));

U_eq = A_I*Diff_U_eq + U_n1;

A_m_list = cell(Np,1);
B_m_list = cell(Np,1);
C_m_list = cell(Np,1);
X_eq = cell(Np,1);
X_eq{1} = X_0;
for i = 1:1:Np
    u_eq = U_eq(Nu*(i - 1) + 1:Nu*i);
    [A_T_eq,B_T_eq,C_T_eq] = getDiscreteMatrix(X_eq{i},u_eq);
    X_eq{i + 1} = getNextState(X_eq{i},u_eq);
%   X_eq{i + 1} = A_T_eq*X_eq{i} + B_T_eq*u_eq + C_T_eq;

    A_m = cell(2,2);
    B_m = cell(2,1);
    C_m = cell(2,1);
    A_m{1,1} = A_T_eq;
    A_m{1,2} = B_T_eq;
    A_m{2,1} = zeros(Nu,Nx);
    A_m{2,2} = eye(Nu,Nu);
    B_m{1,1} = B_T_eq;
    B_m{2,1} = eye(Nu,Nu);
    C_m{1,1} = C_T_eq;
    C_m{2,1} = zeros(Nu,1);

    A_m = cell2mat(A_m);
    B_m = cell2mat(B_m);
    C_m = cell2mat(C_m);

    A_m_list{i} = A_m;
    B_m_list{i} = B_m;
    C_m_list{i} = C_m;
end
X_eq = cell2mat(X_eq);

%% 观测矩阵
H = getParameter('H');

%% 预测矩阵
PHI = cell(Np,1);
THETA = cell(Np,Np);
B_b = cell(Np,Np);
% C_c = cell(Np,1);
temp_A = cell(Np,1);
temp_A{1} = A_m_list{1};
for i = 1:1:Np
    if i >= 2
      temp_A{i} = A_m_list{i}*temp_A{i - 1};
    end
    PHI{i} = H*temp_A{i};
    for j = 1:Np
        B_b{j,j} = B_m_list{j};
%         C_c{j} = C_m_list{j};
        if i >= j
            THETA{i,j} = H*temp_A{i}/temp_A{j};
            if i ~= j
                B_b{i,j} = zeros(Nx + Nu,Nu);
            end
        else
            THETA{i,j} = zeros(Nx,Nx + Nu);
            B_b{i,j} = zeros(Nx + Nu,Nu);
        end
    end
end
PHI = cell2mat(PHI);
THETA = cell2mat(THETA);
B_b = cell2mat(B_b);

% 利用已知轨迹获得修正量
C_c = (THETA)\(X_eq(Nx + 1:(Np + 1)*Nx) - PHI*[X_0;u_n1]) - B_b*Diff_U_eq;

%% 建立约束
u_max = getParameter('u_max');
u_min = getParameter('u_min');
diff_u_max = getParameter('diff_u_max');
diff_u_min = getParameter('diff_u_min');

A_cons = cell(2,1);
b_cons = cell(2,1);

A_cons{1} = A_I;
A_cons{2} = -A_I;

b_cons{1} = kron(ones(Np,1),u_max - u_n1);
b_cons{2} = kron(ones(Np,1),u_n1 - u_min);

lb = kron(ones(Np,1),diff_u_min);
ub = kron(ones(Np,1),diff_u_max);

A_cons = cell2mat(A_cons);
b_cons = cell2mat(b_cons);

%% 建立二次型权重矩阵
%误差权重
Q = getParameter('Q');

%控制增量权重
R = getParameter('R');

%% 二次规划问题
H_Qp = B_b'*THETA'*Q*THETA*B_b + R;
E = PHI*[X_0;u_n1] + THETA*C_c-Y_ref;
f_Qp = E'*Q*THETA*B_b;

% 开始求解
warning off;
[X,fval,flag] = quadprog(H_Qp,f_Qp,A_cons,b_cons,[],[],lb,ub,Diff_U_eq);

Diff_U = X(1:Np*Nu);
Y_opt = PHI*[X_0;u_n1] + THETA*(B_b*Diff_U + C_c);

end
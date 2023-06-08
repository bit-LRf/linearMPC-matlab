% 参数
function param = getParameter(parameterName)
Nx = 1;
Nu = 1;
Np = 20;

mpcInput = 1;
mpcOutput = 1;

T = 0.05;

u_start = 0.01;

u_max = 10;
u_min= -10;
diff_u_max = T*5;
diff_u_min = -T*5;

H= [1 0
    0 1];% 注意：H与Y_ref、Q一定得对应上，不然会报错

q = 1:1:Np;
q = q.^2;

x_gain = 1;
u_gain = 0.001;
Q_gain = 1;

%误差权重
Q = Q_gain*kron(diag(q,0),diag([x_gain,u_gain],0));

r = 1:1:Np;
r = r.^2;

diff_v_gain= 1;
R_gain= 1;

%控制增量权重
R = R_gain*kron(diag(r,0),diag(diff_v_gain,0));

%路径积分收敛系数
mppiGain = 0.01;

    switch parameterName
        case 'mpcInput'
            param = mpcInput;
        case 'mpcOutput'
            param = mpcOutput;
        case 'Nx'
            param = Nx;
        case 'Nu'
            param = Nu;
        case 'Np'
            param = Np;
        case 'T'
            param = T;
        case 'u_start'
            param = u_start;
        case 'u_max'
            param = u_max;
        case 'u_min'
            param = u_min;
        case 'diff_u_max'
            param = diff_u_max;
        case 'diff_u_min'
            param = diff_u_min;
        case 'H'
            param = H;
        case 'Q'
            param = Q;
        case 'R'
            param = R;
        case 'mppiGain'
            param = mppiGain;
    end
end
function [sys,x0,str,ts] = mpcFunction(t,x,u,flag)
switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes;
    case 2
        sys = mdlUpdates(t,x,u);
    case 3
        sys = mdlOutputs(t,x,u);
    case {1,4,9}
        sys = [];
    otherwise
        error(['unhandled flag = ' , num2str(flag)]);
end
end

function [sys,x0,str,ts] = mdlInitializeSizes
tic

%% 设置控制量初值
control_start = getParameter('u_start');
control_memoryFun(control_start,'set');

%% 仿真参数设置
sizes = simsizes;
sizes.NumContStates = 0;
sizes.NumDiscStates = 1;
sizes.NumOutputs = getParameter('mpcOutput');
sizes.NumInputs = getParameter('mpcInput');
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
str = [];
ts = [getParameter('T'),0];
x0 = 0;
end

function sys = mdlUpdates(t,x,u)
sys = x;
end

function sys = mdlOutputs(t,x,u)
% 这是一个基于线性化的MPC控制器。
tic

%% 算法参数
Nx = getParameter('Nx');%状态量的个数
Nu = getParameter('Nu');%控制量的个数
Np = getParameter('Np');%控制时域
T = getParameter('T');

%% 当前状态量，为了防止符号u混淆，先将状态量赋值
% 分别获取
x_n1 = u(1);

% 状态量赋值
state_n1 = x_n1;

%% 控制量
control_n1 = control_memoryFun([],'get');

% 得到平衡点控制增量序列
solution = zeros(Np*Nu,1);

%% 预测一个采样时间后系统的状态量,在本算法中，认为该状态量为零点
state_0 = getNextState(state_n1,control_n1);

%% 参考轨迹生成
observe_ref = getReference(state_0);

%% 计算求解
[solution1,~,~,~] = mpcOptimizer(state_0,control_n1,solution,observe_ref);

[solution2,~,~,~] = mpcOptimizer(state_0,control_n1,solution1,observe_ref);

[solution3,~,~,~] = mpcOptimizer(state_0,control_n1,solution2,observe_ref);

solution = trajectorySynthesis(state_0,control_n1,[solution1,solution2,solution3],observe_ref);

%% 输出
control_output = solution(1) + control_n1;

% 这个才是真的s函数输出，可以自定义增加其他输出
sys = control_output;

% 保存控制量
control_0 = control_output;
control_memoryFun(control_0,'set');

end



%% 其他功能函数
% 控制量存储
function u = control_memoryFun(u_now , flag)
persistent u_memory
switch flag
    case 'set'
        u_memory = u_now;
        u = [];
    case 'get'
        u = u_memory;
    otherwise
        error('unhandled flag');
end
end

% 路径积分控制
function solution = trajectorySynthesis(state_0,control_n1,solution_matrix,observe_ref)
solution = zeros(size(solution_matrix,1),1);

lambda = getParameter('mppiGain');

[row,col] = size(solution_matrix);

expectation = zeros(1,col);
cost = zeros(1,col);

H = getParameter('H');
Q = getParameter('Q');
R = getParameter('R');
T = getParameter('T');

for i = 1:col
    state_seq = kron(zeros(1,row),state_0);
    control_seq = kron(zeros(1,row),control_n1);
    solution_seq = solution_matrix(:,i);
    state_ob = zeros(size(state_seq,1) + size(control_seq,1),row);

    control_seq(:,1) = control_n1 + solution_seq(1);
    state_seq(:,1) = getNextState(state_0,control_seq(:,1),T);

    for j = 1:row - 1
        control_seq(:,j + 1) = control_seq(:,j) + solution_seq(j + 1);

        state_seq(:,j + 1) = getNextState(state_seq(:,j),control_seq(:,j + 1),T);
    end

    state_ob(1:size(state_seq,1),:) = state_seq;
    state_ob(size(state_seq,1) + 1:size(state_seq,1) + size(control_seq,1),:) = control_seq;

    state_ob = H*state_ob;

    state_ob = reshape(state_ob,size(state_ob,1)*size(state_ob,2),1);

    cost(1,i) = (state_ob - observe_ref)'*Q*(state_ob - observe_ref) ...
        + solution_seq'*R*solution_seq;
end

cost_max = max(cost);
cost_min = min(cost);

if(cost_max == cost_min)
    solution = solution_matrix(:,1);
else
    for i = 1:col
        expectation(1,i) = exp(-1/lambda*((cost(1,i) - cost_min)/(cost_max - cost_min)));
        solution = solution + expectation(1,i)*solution_matrix(:,i);
    end
    
    solution = solution/sum(expectation);
end

end
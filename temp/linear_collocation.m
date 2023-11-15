% system
A = [-0.1 2;0 2];
B = [0;1];

n_x = size(A,2);
n_u = size(B,2);

%% control settings
% terminal cost weight
P = eye(n_x);

% state cost weight
Q = eye(n_x);

% control cost weight
R = eye(n_u);

% horizon
t_start = 0;
t_final = 5;

% desired trajectory symbolic function
X_ref_sym = @(t)([0.5 - 0.1*t;0]);

% start state
X_start = [1;0];

%% linear collocation
% number of collocations
N_collocation = 20;

% get collocation parameter
type = 'LG';
param = legendre_lagrange_param(N_collocation - 1,type);

% time at each collocation
t_collocation = param.root*(t_final - t_start)/2 + (t_final + t_start)/2;

% reference at each collocation and boundary
X_ref = zeros(N_collocation*n_x,1);
for k = 1:N_collocation
    X_ref((k - 1)*n_x + 1:k*n_x) = X_ref_sym(t_collocation(k));
end
X_ref_final = X_ref_sym(t_final);

% start matrix
L_start = kron(param.L_start,eye(n_x));

% final matrix
L_final = kron(param.L_final,eye(n_x));

% derivative matrix
D_derivative = kron(param.D_matrix,eye(n_x));
A_full = kron(eye(N_collocation),A);
B_full = kron(eye(N_collocation),B);

% cost function matrix
Q_w_f = kron(diag(param.w,0),Q);
R_w_f = kron(diag(param.w,0),R);

% formulate QP problem
N_variables = N_collocation*n_x + N_collocation*n_u;

% quadradic weight
H = zeros(N_variables,N_variables);
H(1:N_collocation*n_x,1:N_collocation*n_x) = L_final'*P*L_final + (t_final - t_start)/2*Q_w_f;
H(end - N_collocation*n_u + 1:end,end - N_collocation*n_u + 1:end) = (t_final - t_start)/2*R_w_f;

% linear weight(we do not divide 2 here)
f = zeros(1,N_variables);
f(1:N_collocation*n_x) = -2*(X_ref_final'*P*L_final + (t_final - t_start)/2*X_ref'*Q_w_f);

% constraint matrix
G = zeros(n_x + N_collocation*n_x,N_variables);
G(1:n_x,1:N_collocation*n_x) = L_start;
G(end - N_collocation*n_x + 1:end,1:N_collocation*n_x) = D_derivative - (t_final - t_start)/2*A_full;
G(end - N_collocation*n_x + 1:end,end - N_collocation*n_u + 1:end) = -(t_final - t_start)/2*B_full;

% constraint bound
ubg = [X_start;zeros(N_collocation*n_x,1)];
lbg = [X_start;zeros(N_collocation*n_x,1)];

% solve this problem using QP solver
W_opt = quadprog(H,0.5*f,[],[],G,ubg);

%% plot results
X_opt = W_opt(1:N_collocation*n_x);
U_opt = W_opt(N_collocation*n_x + 1:N_collocation*n_x + N_collocation*n_u);

x1 = X_opt(1:2:end);
x2 = X_opt(2:2:end);

u = U_opt;

hold on
plot(t_collocation,X_ref(1:2:end),'Color','blue')
plot(t_collocation,X_ref(2:2:end),'Color','red')
plot(t_collocation,x1,'Marker','o','Color','blue');
plot(t_collocation,x2,'Marker','o','Color','red');
plot(t_collocation,u,'Marker','o','Color','black');
legend('x1 ref','x2 ref','x1','x2','u','Location','best');


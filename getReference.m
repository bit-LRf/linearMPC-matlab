% 参考轨迹生成函数
function Y_ref = getReference(X)
T = getParameter('T');
Np = getParameter('Np');
Nx = getParameter('Nx');
t_vector = linspace(T,T*Np,Np);

x_ref_vector = ones(1,Np).*3;

% 生成参考轨迹向量
Y_ref = zeros(Nx,Np);
Y_ref(1,:) = x_ref_vector;

Y_ref = Y_ref';

end
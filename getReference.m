% 参考轨迹生成函数，最后是一个列向量
function Y_ref = getReference(X)
T = getParameter('T');
Np = getParameter('Np');
Nx = getParameter('Nx');
t_vector = linspace(T,T*Np,Np);

x_ref_vector = ones(1,Np).*3;

% 生成参考轨迹向量
Y_ref = zeros(Nx,Np);
Y_ref(1,:) = x_ref_vector;

Y_ref = reshape(Y_ref,size(Y_ref,1)*size(Y_ref,2),1);

end
function P = legendre_coefficient(N)
% 输入勒让德多项式的阶数:[0,inf),输出勒让德多项式的系数

N = N + 1;

if N == 1
    P = 1;
elseif N == 2
    P = [1 0];
else
    p_n2 = 1;
    p_n1 = [1 0];
    
    for n = 2:N - 1
        P = poly_add(conv(p_n1,[(2*n - 1)/n 0]),-(n - 1)/n*p_n2);
        p_n2 = p_n1;
        p_n1 = P;
    end
end

end
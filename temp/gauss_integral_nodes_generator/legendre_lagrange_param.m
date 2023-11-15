function param = legendre_lagrange_param(N_order,type)
% INPUTS
% N_order: the order of legendre polynomial. the length of root is N_order + 1
% type: the type of legendre-gauss integral point

P_N_plus1 = legendre_coefficient(N_order + 1);

P_N = legendre_coefficient(N_order);

P_N_plus1_dot = polyder(P_N_plus1);

P_N_dot = polyder(P_N);

L = zeros(N_order + 1,N_order + 1);

switch type
    case 'LG'

        root = sort(roots(P_N_plus1));

        for k = 1:N_order + 1
            L(k,:) = 1/(polyval(P_N_plus1_dot,root(k)))*deconv(P_N_plus1,[1 -root(k)]);
        end

    case 'LGR'

        root = sort(roots(poly_add(P_N,P_N_plus1)));

        for k = 1:N_order + 1
            L(k,:) = (1 - root(k))/(2*(N_order + 1)*polyval(P_N,root(k)))*deconv(poly_add(P_N,P_N_plus1),[1 -root(k)]);
        end

    case 'LGL'

        root = sort(roots(conv(P_N_dot,[-1 0 1])));
        
        for k = 1:N_order + 1
            L(k,:) = 1/(N_order*(N_order + 1)*polyval(P_N,root(k)))*deconv(conv(P_N_dot,[1 0 -1]),[1 -root(k)]);
        end

    otherwise

end

L_start = zeros(N_order + 1,0);
L_final = zeros(N_order + 1,0);
D_matrix = zeros(N_order + 1,N_order + 1);
w = zeros(N_order + 1,1);
for k = 1:N_order + 1
    coeff = L(k,:);

    L_start(k) = polyval(coeff,-1);
    L_final(k) = polyval(coeff,1);

    pder = polyder(coeff);
    for r = 1:N_order + 1
        D_matrix(r,k) = polyval(pder,root(r));
    end

    pint = polyint(coeff);
    w(k) = polyval(pint,1.0) - polyval(pint,-1.0);
end

param.root = root;
param.D_matrix = D_matrix;
param.w = w;
param.L = L;
param.L_start = L_start;
param.L_final = L_final;

end
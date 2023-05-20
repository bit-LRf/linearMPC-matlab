% 角度值范围是[-pi,pi]：
function x = mod2pi(x)
n = size(x);
row = n(1);% 行数
col = n(2);% 列数
% x = rem(x,2*pi);
% for i = 1:row
%     for j = 1:col
%         tmp = x(i,j);
%         if tmp < -pi
%             tmp = tmp+2*pi;
%         elseif tmp > pi
%             tmp = tmp-2*pi;
%         end
%         x(i,j) = tmp;
%     end
% end
for i = 1:row
    for j = 1:col
        tmp = x(i,j);
        
        x(i,j) = tmp - pi*fix(tmp/pi);
    end
end
end
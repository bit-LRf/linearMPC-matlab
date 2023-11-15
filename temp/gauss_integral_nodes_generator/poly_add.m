function c = poly_add(a,b)

if length(a) >= length(b)
    c = a + [zeros(1,length(a) - length(b)),b];
else
    c = [zeros(1,length(b) - length(a)),a] + b;
end

end
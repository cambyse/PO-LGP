function K = calcKernelMatrix(k, xi)
%calcKernelMatrix calculates a kernel matrix
    n = size(xi,1);
    K = zeros(n,n);
    for i = 1 : n
        for j = 1 : n
            K(i,j) = k(xi(i),xi(j));
        end;
    end;
end


function k = expKernel(xi, xj, gamma, l)
%expKernel is a exponential Kernel
    k = exp(-(norm(xi-xj)/l)^gamma);
end


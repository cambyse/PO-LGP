function xNext = mpi(mean,var,yBest,i)
    xNext = @(x) cdf('norm',(mean(x)-yBest-500/i.^2)./sqrt(var(x)),0,1);
end


function [x_exp,ymu,ys2,fmu,fs2,hyp,lp] = gpExploration(X,Y,t,hyp)
if (length(hyp)==0)
 hyp.mean = -7;
 ell = 1e-1;
 sn = 1e1;
 hyp.cov = log([ell, sn]);
end
meanfunc = @meanConst; 
covfunc = {@covSEiso};  
likfunc = @likLogistic; 

prior.mean = {{@priorDelta}};
p_ell = {@priorSmoothBox1,log(1e-4),log(5e-1),15};
p_sn = {@priorSmoothBox1,log(1e-3),log(1.5e1),15};
prior.cov = {p_ell,p_sn};

inf = {@infPrior,@infLaplace,prior};

if size(X,1)>4 && (mod(size(X,1),10)==0)
 hyp = minimize(hyp, @gp, -100, inf, meanfunc, covfunc, likfunc, X, Y);
 display(exp(hyp.cov))
end

[ymu, ys2, fmu, fs2, lp] = gp(hyp, inf, meanfunc, covfunc, likfunc, X, Y, t);

[~,idx] = sort(abs(fmu));
idx = idx(1:10);
[~,b]=max(fs2(idx));
x_exp = t(idx(b),:);
end
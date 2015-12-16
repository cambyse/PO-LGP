addpath('/home/englerpr/Dropbox/research/code/util/');
addpath('/home/englerpr/Dropbox/research/code/gpml/');
startup;

ellC = 0.005;
sfC = 1e1;

ellR = 0.0117;
sfR = 0.1682;
snR = 0.012;

trainHyperC = false;
trainHyperR = false;
optPlot = true;
plot_ss = 3;

% define classification hyperparameter
gpc.meanfuncC = @meanConst; hypC.mean = -7;
gpc.covfuncC = {@covSEiso};  hypC.cov = log([ellC, sfC]);
gpc.likfuncC = @likLogistic;
gpc.hypC = hypC;

% define regression hyperparameter
gpr.meanfuncR = [];
gpr.covfuncR = @covSEiso;  hypR.cov = log([ellR; sfR]);
gpr.likfuncR = @likGauss;  hypR.lik = log(snR);
gpr.hypR = hypR;

prior.mean = {{@priorDelta}};
p_ell = {@priorSmoothBox1,log(1e-5),log(2e2),15};
p_sn = {@priorSmoothBox1,log(1e-5),log(1.5e2),15};
prior.cov = {p_ell,p_sn};

gpc.inf = {@infPrior,@infLaplace,prior};

nParam
paramLimit

% define evaluation grid
if (nParam==2)
 [t1 t2] = meshgrid(linspace(paramLimit(1,1),paramLimit(1,2),200),linspace(paramLimit(2,1),paramLimit(2,2),200));
 t = [t1(:) t2(:)];
end

if (nParam==1)
 t = linspace(paramLimit(1,1),paramLimit(1,2),200)';
end

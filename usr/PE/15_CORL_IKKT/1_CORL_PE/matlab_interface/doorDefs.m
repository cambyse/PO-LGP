addpath('/home/englerpr/Dropbox/research/code/gpml/');
startup;

ellC = 0.02;
sfC = 1e1;

ellR = 0.0417;
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
p_ell = {@priorSmoothBox1,log(1e-4),log(2e-1),15};
p_sn = {@priorSmoothBox1,log(1e-3),log(1.5e1),15};
prior.cov = {p_ell,p_sn};

gpc.inf = {@infPrior,@infLaplace,prior};

% define evaluation grid
[t1 t2] = meshgrid(linspace(paramLim(1,1),paramLim(1,2),100),linspace(paramLim(2,1),paramLim(2,2),100));
t = [t1(:) t2(:)];
n_i = size(t,2);

nParam
paramLim

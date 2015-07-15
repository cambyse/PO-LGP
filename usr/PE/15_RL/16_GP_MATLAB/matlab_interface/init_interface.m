addpath('/home/englerpr/Dropbox/research/code/gpml/');
startup;

covfunc = @covSEard; hypR.cov = log([1.1 ; 4.2]);
likfunc = @likGauss; sn = .1; hypR.lik = log(sn);

hypC = {};

nParam
paramLim

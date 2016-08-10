addpath('/home/englerpr/Dropbox/research/code/gpml/');
startup;

covfunc = @covSEard; hypR.cov = log([0.02 ; 9.2]);
likfunc = @likGauss; sn = 0.7; hypR.lik = log(sn);

hypC = {};

nParam
paramLim

addpath('/home/englerpr/Dropbox/research/code/gpml/');
startup;

meanfuncR = @meanConst;
covfuncR = @covSEard; hypR.cov = log([0.02 ; 0.02 ; 4.2]);
likfuncR = @likGauss; sn = 0.7; hypR.lik = log(sn);
hypR.mean = -9;

hypC.mean = -7;
ell = 0.01; sn = 1e1; hypC.cov = log([ell, sn]);

[t1 t2] = meshgrid(linspace(paramLim(1,1),paramLim(1,2),100),linspace(paramLim(2,1),paramLim(2,2),100));
x_grid = [t1(:) t2(:)];

nParam
paramLim

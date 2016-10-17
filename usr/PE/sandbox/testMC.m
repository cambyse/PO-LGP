addpath('../../../../../Dropbox/research/code/gpml');
startup;

fun = @(x) cos(x).*x;
x_gt = [-2*pi:0.1:3]';
y_gt = fun(x_gt);
X = [-2];
Y = fun(X);
covfunc = @covSEard; hyp2.cov = log([1.1 ; 4.2]);
likfunc = @likGauss; sn = .1; hyp2.lik = log(sn);
[m, s2] = gp(hyp2, @infExact, [], covfunc, likfunc, X, Y, x_gt);

figure(1);clf;hold on;
plot(m);

% function to maximize
fun = @(x) exp(-(x(:,1).^2 + x(:,2).^2));
funNoise = @(x) fun(x) + sqrt(0.0001)*randn(size(x,1),1);

% safety function [1:sucess, -1:failure]
safety = @(x) [abs(x(:,1))<3 & abs(x(:,2))<4]*2-1;

% start point
x0 = [2 2];

% grid that defines search space
m = 200
[t1 t2] = ndgrid(linspace(-5,5,m),linspace(-5,5,m));
t = [t1(:) t2(:)];
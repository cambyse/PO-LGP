% function to maximize
fun = @(x) 1-exp(-x.^2);
funNoise = @(x) fun(x) + sqrt(0.0001)*randn(size(x,1),1);

% safety function [1:sucess, -1:failure]
safety = @(x) [x<.8 & x>-.6]*2-1;

% start point
x0=0;

% grid that defines search space
t = ndgrid(-1:0.001:1);
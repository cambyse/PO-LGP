clear;
addpath('gpml');
addpath('problems');
startup;

%% options
nIter = 100;
seed = 1;
e=0;
bOffset = 0.1;
verbose = 2;
optM = 2;

%% Select problem + gp hyperparameter
problem = 'prob_danny';
ellC = 1; sfC = 5;
ellR = 1; sfR = 3e0; snR = 0.11;

% problem = 'prob1';
% ellC = 0.1; sfC = 1e1;
% ellR = 0.1; sfR = 1e0; snR = 0.11;

% problem = 'prob2';
% ellC = 1.5; sfC = 1e1;
% ellR = 0.5; sfR = 1e0; snR = 0.11;

% problem = 'prob3';
% ellC = 1; sfC = 1e1;
% ellR = 1; sfR = 1e0; snR = 0.11;

%% init algorithm
randn('seed',seed)
eval(problem);
n = length(x0);
cbo = conBOpt(n,t,e,bOffset,verbose,optM,ellC,sfC,ellR,sfR,snR);
y = funNoise(x0);
ys = safety(x0);
cbo.addDataPoint(x0,y,ys);

%% run algorithm
for i=2:nIter
 x = cbo.selectNextPoint();
 y = funNoise(x);
 ys = safety(x);
 cboConv = cbo.addDataPoint(x,y,ys);
 cbo.stats();
 cbo.plot();
 if (cboConv)
  break;
 end
end

%% show results
cbo.verbose = 1;
cbo.stats();
cbo.plot();
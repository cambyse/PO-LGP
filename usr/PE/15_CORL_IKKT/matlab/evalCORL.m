clear;
myDefs;
addpath('../../conBOpt/matlab/');
addpath('gpml/');
startup;
folder = ['../data/button4/'];
files = dir([folder,'*.dat']);
names = {files.name};

% load trajectories
for i=1:length(names)
 load([folder,names{i}]);
end
T = size(X0_Demo_X,1);

% load cbo
load([folder,[num2str(Param_id),'_Param_CBO.mat']])
% save('tmp.mat');
%%
% load('tmp.mat')
% plot cbo results
cbo.optM = 2
cbo.selectNextPoint()
cbo.stats();
cbo.plot();
return;
%%
figure(4);clf;hold on;
plot(cbo.statBCR);
plot(cbo.Y,'g.-');

title('best candidate');

%% plot results over time
varList = {
 'Param_Xref';...
 %  'mbXref','mbX';...
 %  'mbFLact','mbUact';...
 %  'phaseXdes','phaseXact';...
 %  'mfUact';...
 };

% evalFun = @(x) sum(sum(x(1:end,15).^2))/T;
% evalFun = @(x) sum(sum(compVel(x,1).^2))/T;
% evalFun = @(x) sum(sum(compAcc(x,1).^2))/T;
evalFun = @(x) sum(sum(compAcc(x,1).^2));

col = {'.r--','sg--','ob'};
for v = 1:size(varList,1)
 figure(v+4);clf;hold on;
 for w = 1:size(varList,2)
  cmb = [];
  for i = 1:id
   str = ['X',num2str(i),'_',varList{v,w}];
   Xv = eval(str);
   cmb = [cmb;evalFun(Xv)];
  end
  plot(1:length(cmb),(cmb),col{w},'MarkerSize',10);
 end
 legend(varList{v,:})
end

%%
% name = 'X1_Param_X';
name = 'X0_Demo_X';

figure(10);clf;hold on;
X = eval(name);
Xref = eval([name,'ref']);
% Xplan = eval([name,'plan']);
load('names.mat');
eps = 0.1;%min(sum(abs(diff(Xplan))))*.9;
idx = sum(abs(diff(X)))>=eps;
X = X(:,idx);
names = {names{idx}};
Xref = Xref(:,idx);
for i=1:size(X,2)
 subplot(size(X,2),1,i); hold on;
 plot(X(:,i));
 plot(Xref(1:1:end,i),'r');
 title(names{(i)})
end
legend('X','Xref');
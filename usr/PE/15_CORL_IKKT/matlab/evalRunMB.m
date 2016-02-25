clear;
myDefs;
folder = ['../data/button4/'];
files = dir([folder,'*.dat']);
names = {files.name};

for i=1:length(names)
 load([folder,names{i}]);
end

T = size(Xbase,1);

%% plot model based results over time
varList = {
 'mfFL';...
%  'mbXref','mbX';...
%  'mbFLact','mbUact';...
%  'phaseXdes','phaseXact';...
%  'mfUact';...
 };

% evalFun = @(x) sum(sum(x(1:end,15).^2))/T;
% evalFun = @(x) sum(sum(compVel(x,1).^2))/T;
% evalFun = @(x) sum(sum(compAcc(x,1).^2))/T;
evalFun = @(x) x(:,3);

col = {'.r--','sg--','ob'};
for v = 1:size(varList,1)
 figure(v);clf;hold on;
 for w = 1:size(varList,2)
  i = 0;
  cmb = [];
  while exist([varList{v,w},num2str(i)])
   Xv = eval([varList{v,w},num2str(i)]);
   cmb = [cmb;evalFun(Xv)];
   i=i+1;
  end
  plot(1:length(cmb),(cmb),col{w},'MarkerSize',10);
 end
 legend(varList{v,:})
end
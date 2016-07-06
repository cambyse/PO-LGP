clear;
myDefs;
folder = ['data/door3/'];
files = dir([folder,'*.dat']);
names = {files.name};

for i=1:length(names)
 load([folder,names{i}]);
end

T = size(mbX,1);

%% plot learning progress
%% plot model free variable over time
cDemo = sum(sum(compVel(Xdemo,1).^2))/T;

varList = {
  'dmpX','dmpXAct';...
  'dmpFLact','';...
};

% evalFun = @(x) sum(sum(x(1:end,15).^2))/T;
% evalFun = @(x) sum(sum(compVel(x,1).^2))/T;
evalFun = @(x) sum(sum(compAcc(x,1).^2))/T;

col = {'r--','g','b'};
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
  plot(1:length(cmb),(cmb),col{w});
 end
 legend(varList{v,:})
end
return;




i = 0;
cDemo = sum(sum(compVel(Xdemo,1).^2))/T;
cDMP = [];
cDMPact = [];
cDMPFL = [];
while exist(['dmpX',num2str(i)])
 X = eval(['dmpX',num2str(i)]);
 cDMP = [cDMP;sum(sum(compVel(X,1).^2))/T];
 X = eval(['dmpXact',num2str(i)]);
 cDMPact = [cDMPact;sum(sum(compVel(X,1).^2))/T];
 X = eval(['dmpFLact',num2str(i)]);
 cDMPFL = [cDMPFL;sum(sum(compVel(X,1).^2))/T];
 i=i+1;
end


figure(1);clf;hold on;
plot(1:length(cDMP),(cDMP),'b.-');
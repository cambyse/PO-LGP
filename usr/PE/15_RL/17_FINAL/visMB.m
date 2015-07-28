clear;
myDefs;
folder = ['data/door2/'];
files = dir([folder,'*.dat']);
names = {files.name};

for i=1:length(names)
 load([folder,names{i}]);
end

T = size(mbX,1);

%% plot learning progress
i = 0;
cDemo = sum(sum(compVel(Xdemo,1).^2))/T;
cMB = [];
cMBact = [];
while exist(['mbX',num2str(i)])
 X = eval(['mbX',num2str(i)]);
 cMB = [cMB;sum(sum(compVel(X,1).^2))/T];
 X = eval(['mbXact',num2str(i)]);
 cMBact = [cMBact;sum(sum(compVel(X,1).^2))/T];
 i=i+1;
end

i=0;
cPhase = [];
cPhaseAct = [];
while exist(['phaseX',num2str(i)])
 X = eval(['phaseX',num2str(i)]);
 cPhase = [cPhase;sum(sum(compVel(X,1).^2))/T];
 X = eval(['phaseXact',num2str(i)]);
 cPhaseAct = [cPhaseAct;sum(sum(compVel(X,1).^2))/T];
 i=i+1;
end

figure(1);clf;hold on;
plot(0,(cDemo),'.g');
plot(1:length(cMB),(cMB));
plot(1:length(cMBact),(cMBact),'r');

plot(length(cMB)+1:length(cMB)+length(cPhase),(cPhase),'r--');
plot(length(cMB)+1:length(cMB)+length(cPhase),(cPhaseAct),'b--');
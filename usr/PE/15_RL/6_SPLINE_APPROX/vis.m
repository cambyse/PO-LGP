clear;
myDefs;
files = dir('data/*.dat');
names = {files.name};

for i=1:length(names)
 load(['data/',names{i}]);
end
t_b = linspace(0,1,size(beta,1));
t_x = linspace(0,1,size(X,1));
for i =1:size(X,2)
 figure(i);clf;hold on;
 plot(t_x,X(:,i));
 plot(t_x,Xres(:,i),'r');
 plot(t_b,beta(:,i),'g.');
end
legend('reference','result')

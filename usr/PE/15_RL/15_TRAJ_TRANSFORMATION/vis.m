clear;
myDefs;
files = dir('plots/*.dat');
names = {files.name};

for i=1:length(names)
 load(['plots/',names{i}]);
end

figure(1);clf;hold on;
for i =1:size(Xact,2)
 if norm(Xact(:,i)-Xdes(:,i))>1e-1
 plot(Xact(:,i));
 plot(Xdes(:,i),'g');
 end
end
